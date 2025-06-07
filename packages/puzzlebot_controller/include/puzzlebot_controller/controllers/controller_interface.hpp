#ifndef PUZZLEBOT_CONTROLLER__CONTROLLERS__CONTROLLER_INTERFACE_HPP_
#define PUZZLEBOT_CONTROLLER__CONTROLLERS__CONTROLLER_INTERFACE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <vector>
#include <cmath>

namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        class ControllerInterface {
            public:
                double linear_speed_;
                double angular_speed_;
                int path_index_ = 0;
                tf2_ros::Buffer* tf_buffer_;  // pointer, not owned

                ControllerInterface(double linear_speed, double angular_speed, 
                                    tf2_ros::Buffer* tf_buffer)
                : tf_buffer_(tf_buffer)
                { 
                    linear_speed_ = linear_speed; 
                    angular_speed_ = angular_speed;
                }

                virtual ~ControllerInterface() = default;
                
                virtual bool computeCommand(
                    const geometry_msgs::msg::PoseStamped& current_pose,
                    const std::vector<geometry_msgs::msg::PoseStamped>& path,
                    geometry_msgs::msg::Twist::SharedPtr cmd
                ) = 0;
                
                void resetIndex() { setPathIndex(0); }
                
                // SETTER AND GETTERS
                void setPathIndex(int new_index) { path_index_ = new_index; } 
                int getPathIndex() const { return path_index_; }
        };
    }
}


#endif