#include "puzzlebot_controller/controllers/pure_pursuit.hpp"
#include <cmath>


namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        // PurePursuitController::PurePursuitController(double linear_speed, double angular_speed, double lookahead_distance, double orientation_tolerance) : 
        // ControllerInterface(linear_speed, angular_speed), lookahead_distance_(lookahead_distance), lookahead_delta_(0.0), orientation_tolerance_(orientation_tolerance) {}

        PurePursuitController::PurePursuitController(double linear_speed, double angular_speed, double lookahead_distance, double orientation_tolerance, tf2_ros::Buffer* tf_buffer) :
                    ControllerInterface(linear_speed, angular_speed, tf_buffer),
                    lookahead_distance_(lookahead_distance), 
                    lookahead_delta_(0.0), 
                    orientation_tolerance_(orientation_tolerance) {}

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
                if (dist >= lookahead_distance_ - lookahead_delta_) {
                    lookahead = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
                    path_index_ = i;
                    break;
                }
            }

            // If no lookahead point, we are at the end -> align to final pose
            if (lookahead == nullptr || orientation_correction_) {

                if (lookahead_delta_ < lookahead_distance_ * 0.6 && !orientation_correction_) {
                    lookahead_delta_ += lookahead_distance_ * 0.1;
                    return false;  // Increment lookahead distance
                }

                orientation_correction_ = true;
                // const auto& goal_pose = path.back().pose;
                geometry_msgs::msg::PoseStamped goal_pose = path.back();
                goal_pose.header.stamp = rclcpp::Time(0);

                geometry_msgs::msg::PoseStamped goal_pose_in_base;
                try {
                    tf_buffer_->transform(goal_pose, goal_pose_in_base, "base_link");
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(rclcpp::get_logger("PurePursuit"), "Transform failed: %s", ex.what());
                    return false;
                }

                double yaw_goal = tf2::getYaw(goal_pose_in_base.pose.orientation);
                double yaw_current = 0.0;  // robot's own yaw in base_link is always 0
                double yaw_error = angles::normalize_angle(yaw_goal);

                // If orientation is aligned, stop
                if (std::abs(yaw_error) < orientation_tolerance_) {
                    orientation_correction_ = false;
                    lookahead_delta_ = 0.0;  // Reset lookahead distance
                    cmd->linear.x = 0.0;
                    cmd->angular.z = 0.0;
                    return true;  // Goal fully reached
                }

                // Otherwise, rotate in place
                cmd->linear.x = 0.0;
                cmd->angular.z = std::clamp(yaw_error, -angular_speed_, angular_speed_);
                return false;
            }
        

            lookahead->header.stamp = rclcpp::Time(0);// Use current pose timestamp
            geometry_msgs::msg::PoseStamped lookahead_in_base;
            try {
                tf_buffer_->transform(*lookahead, lookahead_in_base, "base_link");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(rclcpp::get_logger("PurePursuit"), "Transform failed on controller: %s", ex.what());
                return false;
            }

            // Calculate curvature and angular velocity
            double effective_lookahead = lookahead_distance_ - lookahead_delta_;
            if (effective_lookahead < 1e-4) {
                RCLCPP_WARN(rclcpp::get_logger("PurePursuit"), "Effective lookahead too small, skipping command.");
                return false;
            }

            // std::cout << "[" << path_index_ << "] " <<  lookahead->pose.position.x << ", "<< lookahead->pose.position.y << std::endl;
            
            double curvature = 2.0 * lookahead_in_base.pose.position.y / (effective_lookahead * effective_lookahead);

            // double curvature = 2.0 * y / (effective_lookahead * effective_lookahead);
            double angular_z = linear_speed_ * curvature;

            cmd->linear.x = std::fabs(angular_z) > angular_speed_ ? linear_speed_ * 0.2 : linear_speed_;
            cmd->angular.z = std::clamp(angular_z, -angular_speed_, angular_speed_);
            
            return false;
        }
    }
}