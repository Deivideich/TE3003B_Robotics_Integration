#include "puzzlebot_controller/controllers/pure_pursuit.hpp"
#include <cmath>
namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        PurePursuitController::PurePursuitController(double linear_speed, double angular_speed, double lookahead_distance, double orientation_tolerance) : 
        ControllerInterface(linear_speed, angular_speed), lookahead_distance_(lookahead_distance), orientation_tolerance_(orientation_tolerance) {}

        bool PurePursuitController::computeCommand(
            const geometry_msgs::msg::PoseStamped& current_pose,
            const std::vector<geometry_msgs::msg::PoseStamped>& path,
            geometry_msgs::msg::Twist::SharedPtr cmd) 
        {
            // Find lookahead point
            std::shared_ptr<geometry_msgs::msg::PoseStamped> lookahead = nullptr;
            for (int i = path_index_; i < path.size(); i++) {
                const auto& pose = path[i];
                double dx = pose.pose.position.x - current_pose.pose.position.x;
                double dy = pose.pose.position.y - current_pose.pose.position.y;
                double dist = std::hypot(dx, dy);
                if (dist >= lookahead_distance_ ) {
                    lookahead = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
                    path_index_ = i;
                    break;
                }
            }

            // If no lookahead point, we are at the end -> align to final pose
            if (lookahead == nullptr || orientation_correction_) {
                orientation_correction_ = true;
                const auto& goal_pose = path.back().pose;

                // Compute angle difference
                tf2::Quaternion q_current, q_goal;
                tf2::fromMsg(current_pose.pose.orientation, q_current);
                tf2::fromMsg(goal_pose.orientation, q_goal);

                double yaw_current = tf2::getYaw(q_current);
                double yaw_goal = tf2::getYaw(q_goal);
                double yaw_error = angles::shortest_angular_distance(yaw_current, yaw_goal);

                // If orientation is aligned, stop
                if (std::abs(yaw_error) < orientation_tolerance_) {
                    orientation_correction_ = false;
                    cmd->linear.x = 0.0;
                    cmd->angular.z = 0.0;
                    return true;  // Goal fully reached
                }

                // Otherwise, rotate in place
                cmd->linear.x = 0.0;
                cmd->angular.z = std::clamp(yaw_error, -angular_speed_, angular_speed_);
                return false;
            }

            // Transform goal to robot frame
            tf2::Transform tf_robot;
            tf2::fromMsg(current_pose.pose, tf_robot);

            tf2::Vector3 goal_vec(lookahead->pose.position.x, lookahead->pose.position.y, 0);
            tf2::Vector3 goal_rel = tf_robot.inverse() * goal_vec;

            double y = goal_rel.y();

            // Calculate curvature and angular velocity
            double curvature = 2 * y / (lookahead_distance_  * lookahead_distance_   );
            double angular_z = linear_speed_ * curvature;

            cmd->linear.x = std::fabs(angular_z) > angular_speed_ ? linear_speed_ * 0.2 : linear_speed_;
            cmd->angular.z = std::clamp(angular_z, -angular_speed_, angular_speed_);
            
            return false;
        }
    }
}