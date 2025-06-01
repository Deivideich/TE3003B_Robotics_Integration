#include "puzzlebot_controller/bug_controllers/bug0_controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>

namespace puzzlebot_controllers 
{
namespace bug_controllers 
{

Bug0Controller::Bug0Controller(double linear_speed, double angular_speed, double desired_distance)
: BugControllerInterface(linear_speed)
{
    state_ = IDLE;
    obstacle_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    angular_speed_ = angular_speed;
    desired_distance_ = desired_distance;
}

bool Bug0Controller::computeCommand(
    const geometry_msgs::msg::PoseStamped& current_pose,
    geometry_msgs::msg::Twist::SharedPtr cmd)
{
    if (!cmd) return false;

    switch (state_)
    {
        case IDLE:
            state_ = NAV_TO_GOAL;
            break;

        case NAV_TO_GOAL:
            if (isLineToGoalBLocked(current_pose))
            {
                state_ = AVOIDING_OBSTACLE;
                return computeCommand(current_pose, cmd); // re-evaluate with new state
            }

            // Drive directly toward goal
            *cmd = driveToPose(getGoalPose(), current_pose);
            if (isCloseToGoal(current_pose))
                state_ = REACHED;

            break;

        case AVOIDING_OBSTACLE:
        {
            geometry_msgs::msg::PoseStamped tangent_pose = computeNextPose();
            *cmd = driveToPose(tangent_pose, current_pose);

            // After a short time or distance, recheck for clear line
            if (!isLineToGoalBLocked(current_pose))
                state_ = NAV_TO_GOAL;
        }
        break;

        case REACHED:
            cmd->linear.x = 0.0;
            cmd->angular.z = 0.0;
            return false;
    }

    return true;
}

bool Bug0Controller::isLineToGoalBLocked(const geometry_msgs::msg::PoseStamped& current_pose)
{   
    auto map = getLocalMap();
    if (!map) return false;

    const nav_msgs::msg::OccupancyGrid& grid = *map;

    double resolution = grid.info.resolution;
    double origin_x = grid.info.origin.position.x;
    double origin_y = grid.info.origin.position.y;
    int width = grid.info.width;
    int height = grid.info.height;

    const auto goal = getGoalPose();

    // Convert world coordinates to map indices
    auto toGrid = [&](double wx, double wy) -> std::pair<int, int> {
        int gx = static_cast<int>((wx - origin_x) / resolution);
        int gy = static_cast<int>((wy - origin_y) / resolution);
        return {gx, gy};
    };

    auto [x0, y0] = toGrid(current_pose.pose.position.x, current_pose.pose.position.y);
    auto [x1, y1] = toGrid(goal.pose.position.x, goal.pose.position.y);

    // Bresenham's algorithm for line traversal
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    int x = x0;
    int y = y0;

    while (true) {
        // Check boundaries
        if (x < 0 || x >= static_cast<int>(width) || y < 0 || y >= static_cast<int>(height)) break;

        int index = y * width + x;
        if (grid.data[index] > 50) {
            // Obstacle found
            obstacle_pose->pose.position.x = x * resolution + origin_x;
            obstacle_pose->pose.position.y = y * resolution + origin_y;
            obstacle_pose->header = map->header;
            return true;
        }

        if (x == x1 && y == y1) break;

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }

    return false;
}


geometry_msgs::msg::PoseStamped Bug0Controller::computeNextPose()
{
    geometry_msgs::msg::PoseStamped tangent_pose;

    const double ox = obstacle_pose->pose.position.x;
    const double oy = obstacle_pose->pose.position.y;

    // Tangent vector (perpendicular to obstacle direction)
    double tangent_x = -oy;
    double tangent_y = ox;
    double norm = std::hypot(tangent_x, tangent_y);

    tangent_x /= norm;
    tangent_y /= norm;

    double target_x = ox + tangent_x * 0.5; // advance 0.5 m along tangent
    double target_y = oy + tangent_y * 0.5;

    // Offset in tangent frame to keep distance from object (to the left side of tangent)
    double perp_x = -tangent_y;
    double perp_y = tangent_x;

    target_x += perp_x * desired_distance_;
    target_y += perp_y * desired_distance_;

    tangent_pose.header.frame_id = "base_link";
    tangent_pose.pose.position.x = target_x;
    tangent_pose.pose.position.y = target_y;

    tf2::Quaternion q;
    q.setRPY(0, 0, std::atan2(tangent_y, tangent_x));
    tangent_pose.pose.orientation = tf2::toMsg(q);

    return tangent_pose;
}

geometry_msgs::msg::Twist Bug0Controller::driveToPose(
    const geometry_msgs::msg::PoseStamped& target_pose,
    const geometry_msgs::msg::PoseStamped& current_pose)
{
    geometry_msgs::msg::Twist cmd;

    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    double angle_to_target = std::atan2(dy, dx);

    double yaw = tf2::getYaw(current_pose.pose.orientation);
    double angular_error = angle_to_target - yaw;

    // Normalize angle
    while (angular_error > M_PI) angular_error -= 2 * M_PI;
    while (angular_error < -M_PI) angular_error += 2 * M_PI;

    if (std::abs(angular_error) > 0.1)
    {
        cmd.angular.z = angular_speed_ * float((angular_error) / fabs(angular_error));
    }
    else
    {
        cmd.linear.x = getLinearSpeed();
        cmd.angular.z = 0.0;
    }

    return cmd;
}

bool Bug0Controller::isCloseToGoal(const geometry_msgs::msg::PoseStamped& current_pose)
{
    const geometry_msgs::msg::PoseStamped goal = getGoalPose();
    double dx = goal.pose.position.x - current_pose.pose.position.x;
    double dy = goal.pose.position.y - current_pose.pose.position.y;
    return std::hypot(dx, dy) < 0.2;
}

geometry_msgs::msg::PoseStamped Bug0Controller::transfromToBaselink(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    // Currently unused. Stub for future TF-based transformation if needed.
    return *pose;
}

}  // namespace bug_controllers
}  // namespace puzzlebot_controllers
