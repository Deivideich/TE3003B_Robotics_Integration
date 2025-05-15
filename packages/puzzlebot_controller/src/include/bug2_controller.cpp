#include "puzzlebot_controller/controllers/bug2_controller.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>

double angle_diff(double a, double b){
    double diff = a - b;
    while (diff > M_PI) diff -= 2*M_PI;
    while (diff < -M_PI) diff += 2*M_PI;
    
    return diff;
}

namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        Bug2Controller::Bug2Controller(double linear_speed)
            : linear_speed_(linear_speed), min_distance_to_goal_(std::numeric_limits<double>::max()) {}

        bool Bug2Controller::computeCommand(
            const geometry_msgs::msg::PoseStamped& current_pose,
            const std::vector<geometry_msgs::msg::PoseStamped>& path,
            geometry_msgs::msg::Twist::SharedPtr cmd) {

            geometry_msgs::msg::Point current_pos = current_pose.pose.position;

            bool on_mline = onMLine(current_pos);
            bool closer = closerToGoal(current_pos);

            left_mline_ = !on_mline ? true : left_mline_;

            if (left_mline_ && state_ == REACHED) return true;  // Done with bug mode


            const double front_angle = 0.0;
            const double right_angle = -M_PI_2;
            const double angle_range = M_PI / 2;
            const double check_distance = 0.50; // meters

            bool front_blocked = isDirectionBlocked(current_pose, front_angle - angle_range, front_angle + angle_range, angle_range / 8, check_distance, false);
            bool right_blocked = isDirectionBlocked(current_pose, right_angle - angle_range, right_angle + angle_range, angle_range / 8, check_distance, false);
            
            tf2::Quaternion q(
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w
            );
            
            double theta = tf2::getYaw(q);

            if (std::abs(angle_diff(desired_angle_, theta)) < 0.1){
                state_ = state_ == TURNING_RIGHT ? REACHED : IDLE;
            }
            std::cout << "Front Blocked: " << front_blocked << ", Right Blocked: " << right_blocked << std::endl;

            if (state_ == IDLE) {
                if (object_on_right && !right_blocked) {
                    std::cout << "Turn RIGHT" << std::endl;
                    state_ = TURNING_RIGHT;
                    desired_angle_ = theta - M_PI_2;
                } else if (object_on_right && !right_blocked && !front_blocked){
                    std::cout << "Forward" << std::endl;
                } else {
                    std::cout << "Turn LEFT" << std::endl;
                    state_ = TURNING_LEFT;
                    desired_angle_ = theta + M_PI_2;
                }
            }

            switch (state_)
            {
            case TURNING_RIGHT:
                cmd->linear.x = 0.15;
                cmd->angular.z = -0.3;
                break;
            case TURNING_LEFT:
                if (!object_on_right) object_on_right = true;
                cmd->linear.x = 0.0;
                cmd->angular.z = 0.3;
                break;
            default:
                cmd->linear.x = 0.1;
                cmd->angular.z = 0.0;
                break;
            }
            

            return false;
        }

        bool Bug2Controller::isDirectionBlocked(
            const geometry_msgs::msg::PoseStamped& pose, 
            double angle_min, double angle_max, double angle_step, 
            double distance, bool useTheta = true) 
        {
            if (!local_map_) return false;
        
            // Robot pose
            double robot_x = pose.pose.position.x;
            double robot_y = pose.pose.position.y;
        
            tf2::Quaternion q(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            );
            double theta = tf2::getYaw(q);
            double step_size = local_map_->info.resolution; // Use map resolution as step size
            // Sweep through angles relative to robot heading
            for (double angle = angle_min; angle <= angle_max; angle += angle_step) {
                double check_angle = angle;
        
                for (double step = 0.0; step <= distance; step += step_size) {
                    double check_x = robot_x + step * std::cos(check_angle);
                    double check_y = robot_y + step * std::sin(check_angle);
        
                    // Convert to map coordinates
                    int mx = (check_x - local_map_->info.origin.position.x) / local_map_->info.resolution;
                    int my = (check_y - local_map_->info.origin.position.y) / local_map_->info.resolution;
        
                    if (mx < 0 || mx >= static_cast<int>(local_map_->info.width) ||
                        my < 0 || my >= static_cast<int>(local_map_->info.height)) {
                        continue;  // Out of bounds, skip
                    }
        
                    int index = my * local_map_->info.width + mx;
        
                    if (index >= 0 && index < static_cast<int>(local_map_->data.size()) &&
                        local_map_->data[index] > 50) {
                        return true; // Occupied cell detected
                    }
                }
            }
        
            return false; // No obstacle found in the scanned area
        }
        
        

        void Bug2Controller::setMLine(const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& goal) {
            mline_start_ = start;
            mline_goal_ = goal;
        }

        void Bug2Controller::reset(){
            min_distance_to_goal_ = std::numeric_limits<double>::max();
            left_mline_ = false;
            state_ = IDLE;
        }

        bool Bug2Controller::onMLine(const geometry_msgs::msg::Point& current_pos) {
            // Check if the current position is on the M-line
            double dx = mline_goal_.x - mline_start_.x;
            double dy = mline_goal_.y - mline_start_.y;
            double cross_product = (current_pos.x - mline_start_.x) * dy - (current_pos.y - mline_start_.y) * dx;
            return std::abs(cross_product) < 0.1; // Tolerance for being on the line
        }

        bool Bug2Controller::closerToGoal(const geometry_msgs::msg::Point& current_pos) {
            // Check if the robot is closer to the goal than before
            double distance_to_goal = std::hypot(
                current_pos.x - mline_goal_.x,
                current_pos.y - mline_goal_.y);

            if (distance_to_goal < min_distance_to_goal_) {
                min_distance_to_goal_ = distance_to_goal;
                return true;
            }
            return false;
        }
    }
}