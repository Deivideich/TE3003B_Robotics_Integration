#ifndef PUZZLEBOT_CONTROLLER__CONTROLLERS__PID_CONTROLLER_HPP_
#define PUZZLEBOT_CONTROLLER__CONTROLLERS__PID_CONTROLLER_HPP_

#include "puzzlebot_controller/controllers/controller_interface.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>


namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        class PIDController : public ControllerInterface {
            private:
                double kP_;
                double kD_;
                double kI_;
            public:
                PIDController(double linear_speed, double kP, double kD, double kI);
                
                geometry_msgs::msg::Twist computeCommand(
                    const geometry_msgs::msg::PoseStamped& current_pose,
                    const std::vector<geometry_msgs::msg::PoseStamped>& path) override;
        };
    }
}

#endif