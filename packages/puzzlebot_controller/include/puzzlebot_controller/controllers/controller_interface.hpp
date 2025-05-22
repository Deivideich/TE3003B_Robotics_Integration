#ifndef PUZZLEBOT_CONTROLLER__CONTROLLERS__CONTROLLER_INTERFACE_HPP_
#define PUZZLEBOT_CONTROLLER__CONTROLLERS__CONTROLLER_INTERFACE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        class ControllerInterface {
            public:
                double linear_speed_;
                int path_index_ = 0;
                ControllerInterface(double linear_speed) { linear_speed_ = linear_speed; }

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