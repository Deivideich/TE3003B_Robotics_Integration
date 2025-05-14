#include "puzzlebot_controller/controllers/pid_controller.hpp"

namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        PIDController::PIDController(double linear_speed, double kP, double kD, double kI) : 
                             kP_(kP), kD_(kD), kI_(kI), ControllerInterface(linear_speed) {}

        geometry_msgs::msg::Twist PIDController::computeCommand(
            const geometry_msgs::msg::PoseStamped& current_pose,
            const std::vector<geometry_msgs::msg::PoseStamped>& path) 
        {
            geometry_msgs::msg::Twist cmd;
            return cmd;
        }
    }
}

