#include "puzzlebot_controller/bug_controllers/bug0_controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>

namespace puzzlebot_controllers 
{
namespace bug_controllers 
{

Bug0Controller::Bug0Controller(double linear_speed, double angular_speed, double desired_distance, double angle_delta, int dfs_scale)
: BugControllerInterface(linear_speed)
{
    state_ = IDLE;
    obstacle_blob_ = new std::vector<geometry_msgs::msg::PointStamped>();
    angular_speed_ = angular_speed;
    desired_distance_ = desired_distance;
    angle_delta_ = angle_delta; 
    dfs_scale_ = dfs_scale;
}

bool Bug0Controller::computeCommand(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& goal_pose,
    geometry_msgs::msg::Twist::SharedPtr cmd)
{
    if (!cmd) return false;

    std::cout << "BUG state " << state_ << std::endl;
    switch (state_)
    {
        case IDLE:
            state_ = NAV_TO_GOAL;
            break;

        case NAV_TO_GOAL:
            if (isLineToGoalBlocked(current_pose, goal_pose))
            {
                state_ = AVOIDING_OBSTACLE;
                return computeCommand(current_pose, goal_pose, cmd); // re-evaluate with new state
            }

            // Drive directly toward goal
            *cmd = driveToPose(goal_pose, current_pose);
            if (isCloseToGoal(current_pose, goal_pose))
                state_ = REACHED;

            break;

        case AVOIDING_OBSTACLE:
        {
            
            geometry_msgs::msg::PoseStamped::SharedPtr tangent_pose = computeNextPose();
            if (tangent_pose == nullptr) return false;
            *cmd = driveToPose(*tangent_pose, current_pose);
            // After a short time or distance, recheck for clear line
            if (!isLineToGoalBlocked(current_pose, goal_pose))
                state_ = NAV_TO_GOAL;
        }
        break;

        case REACHED:
            cmd->linear.x = 0.0;
            cmd->angular.z = 0.0;
            return true;
    }

    return false;
}

std::pair<int,int> Bug0Controller::toGrid(double wx, double wy, const nav_msgs::msg::OccupancyGrid& grid){
    int gx = static_cast<int>((wx - grid.info.origin.position.x) / grid.info.resolution);
    int gy = static_cast<int>((wy - grid.info.origin.position.y) / grid.info.resolution);
    return {gx, gy};
}

bool Bug0Controller::isLineToGoalBlocked(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& goal_pose)
{
    auto map = getLocalMap();
    if (!map) {
        std::cout << "No local map received" << std::endl;
        return false;
    }

    const nav_msgs::msg::OccupancyGrid& grid = *map;
    double resolution = grid.info.resolution;
    double origin_x = grid.info.origin.position.x;
    double origin_y = grid.info.origin.position.y;
    int width = grid.info.width;
    int height = grid.info.height;

    double dx = goal_pose.pose.position.x - current_pose.pose.position.x;
    double dy = goal_pose.pose.position.y - current_pose.pose.position.y;
    double goal_angle = std::atan2(dy, dx);
    double goal_distance = std::hypot(dx, dy);

    const int num_rays = 15;
    double min_angle = goal_angle - angle_delta_;
    double max_angle = goal_angle + angle_delta_;

    for (int i = 0; i < num_rays; ++i) {
        double angle = min_angle + i * (max_angle - min_angle) / (num_rays - 1);
        double end_x = current_pose.pose.position.x + goal_distance * std::cos(angle);
        double end_y = current_pose.pose.position.y + goal_distance * std::sin(angle);

        // Convert to map indices
        auto [x0, y0] = toGrid(current_pose.pose.position.x, current_pose.pose.position.y, grid);
        auto [x1, y1] = toGrid(end_x, end_y, grid);

        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        int x = x0, y = y0;


        obstacle_blob_->clear();

        while (true) {
            if ((x < 0 || x >= width) && (y < 0 || y >= height))
                break;

            int index = y * width + x;
            if (grid.data[index] > 50) {  // Occupied
                // Optionally store obstacle point
                geometry_msgs::msg::PointStamped object_point;
                object_point.header = grid.header;
                object_point.point.x = x * resolution + origin_x;
                object_point.point.y = y * resolution + origin_y;
                obstacle_blob_->push_back(object_point);
            }

            if (x == x1 && y == y1) break;

            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x += sx; }
            if (e2 <= dx) { err += dx; y += sy; }
        }
    }

    return !obstacle_blob_->empty();
}


void Bug0Controller::getDFSObject(const nav_msgs::msg::OccupancyGrid& grid, std::pair<int,int> coord, std::unordered_set<std::pair<int,int>, hashFunction>& visited, std::vector<std::pair<int,int>>& object_corners){
    if (visited.find(coord) != visited.end()) 
        return;
    
    visited.insert(coord);

    int num_neighbours = 0;
    for (int scale = 1; scale <= dfs_scale_; scale++){
        for (auto direction : directions_){
            auto new_coord = std::make_pair(coord.first + direction.first * scale, coord.second + direction.second * scale);

            if (new_coord.first < 0 || new_coord.first >= grid.info.width  || new_coord.second < 0 || new_coord.second >= grid.info.height) continue;

            int index = new_coord.second * grid.info.width + new_coord.first;

            if (visited.find(new_coord) == visited.end() && grid.data[index] > 50)
            {    
                num_neighbours++;
                getDFSObject(grid, new_coord, visited, object_corners);
            }
        }
    }

    if (num_neighbours == 0) object_corners.push_back(coord);
}


geometry_msgs::msg::PoseStamped::SharedPtr Bug0Controller::computeNextPose()
{
    auto map = getLocalMap();
    if (!map || obstacle_blob_->empty()) return nullptr;

    const nav_msgs::msg::OccupancyGrid& grid = *map;

    
    std::unordered_set<std::pair<int,int>, hashFunction> visited;
    std::vector<std::pair<int,int>> object_corners;
    
    for (const auto& obstacle_point : *obstacle_blob_) {
        auto next_init_coord = toGrid(obstacle_point.point.x, obstacle_point.point.y, grid);
        if (visited.find(next_init_coord) != visited.end()) continue;
        getDFSObject(grid, next_init_coord, visited, object_corners);
    }

    if (object_corners.empty()) return nullptr;

    // === Find top-right and bottom-left corners ===
    std::pair<int, int> top_right = object_corners.front();
    std::pair<int, int> bottom_left = object_corners.front();

    for (const auto& corner : object_corners) {
        if (corner.first >= top_right.first && corner.second >= top_right.second)
            top_right = corner;
        if (corner.first <= bottom_left.first && corner.second <= bottom_left.second)
            bottom_left = corner;
    }

    // === Compute tangent angle ===
    double dx = top_right.first - bottom_left.first;
    double dy = top_right.second - bottom_left.second;
    double angle = std::atan2(dy, dx);

    // === Create frame at top-right corner ===
    double origin_x = grid.info.origin.position.x;
    double origin_y = grid.info.origin.position.y;
    double res = grid.info.resolution;

    double base_x = top_right.first * res + origin_x;
    double base_y = top_right.second * res + origin_y;

    tf2::Transform object_tf;
    object_tf.setOrigin(tf2::Vector3(base_x, base_y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    object_tf.setRotation(q);

    // === Offset forward by desired_distance_ along x-axis ===
    tf2::Vector3 offset(desired_distance_, 0.0, 0.0);
    tf2::Vector3 target = object_tf * offset;

    auto goal = std::make_shared<geometry_msgs::msg::PoseStamped>();
    goal->header = grid.header;
    goal->pose.position.x = target.x();
    goal->pose.position.y = target.y();
    goal->pose.position.z = 0.0;
    goal->pose.orientation = tf2::toMsg(q);  // maintain orientation

    // obstacle_blob_->clear();

    return goal;

    // geometry_msgs::msg::PoseStamped tangent_pose;

    // const double ox = 0;//obstacle_pose->pose.position.x;
    // const double oy = 0;//obstacle_pose->pose.position.y;

    // // Tangent vector (perpendicular to obstacle direction)
    // double tangent_x = -oy;
    // double tangent_y = ox;
    // double norm = std::hypot(tangent_x, tangent_y);

    // tangent_x /= norm;
    // tangent_y /= norm;

    // double target_x = ox + tangent_x * 0.5; // advance 0.5 m along tangent
    // double target_y = oy + tangent_y * 0.5;

    // // Offset in tangent frame to keep distance from object (to the left side of tangent)
    // double perp_x = -tangent_y;
    // double perp_y = tangent_x;

    // target_x += perp_x * desired_distance_;
    // target_y += perp_y * desired_distance_;

    // tangent_pose.header.frame_id = "base_link";
    // tangent_pose.pose.position.x = target_x;
    // tangent_pose.pose.position.y = target_y;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, std::atan2(tangent_y, tangent_x));
    // tangent_pose.pose.orientation = tf2::toMsg(q);

    // return tangent_pose;
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

    setControlPose(target_pose);

    return cmd;
}

bool Bug0Controller::isCloseToGoal(const geometry_msgs::msg::PoseStamped& current_pose, const geometry_msgs::msg::PoseStamped& goal_pose)
{
    const geometry_msgs::msg::PoseStamped goal = goal_pose; //getGoalPose();
    double dx = goal.pose.position.x - current_pose.pose.position.x;
    double dy = goal.pose.position.y - current_pose.pose.position.y;
    return std::hypot(dx, dy) < 0.2;
}

}  // namespace bug_controllers
}  // namespace puzzlebot_controllers
