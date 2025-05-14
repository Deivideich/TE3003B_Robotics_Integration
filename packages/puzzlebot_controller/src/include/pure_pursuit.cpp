#include "puzzlebot_controller/controllers/pure_pursuit.hpp"

#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        PurePursuitController::PurePursuitController(double linear_speed, double lookahead_distance) : 
        ControllerInterface(linear_speed), lookahead_distance_(lookahead_distance) {}

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

            // Stop if we are at the end
            if (lookahead == nullptr) return true;

            // Transform goal to robot frame
            tf2::Transform tf_robot;
            tf2::fromMsg(current_pose.pose, tf_robot);

            tf2::Vector3 goal_vec(lookahead->pose.position.x, lookahead->pose.position.y, 0);
            tf2::Vector3 goal_rel = tf_robot.inverse() * goal_vec;

            double y = goal_rel.y();

            // Calculate curvature and angular velocity
            double curvature = 2 * y / (lookahead_distance_  * lookahead_distance_   );
            double angular_z = linear_speed_ * curvature;

            cmd->linear.x = linear_speed_;
            cmd->angular.z = angular_z;
            
            return false;
        }
    }
}