#ifndef PUZZLEBOT_CONTROLLER__CONTROLLERS__PURE_PURSUIT_HPP_
#define PUZZLEBOT_CONTROLLER__CONTROLLERS__PURE_PURSUIT_HPP_

#include "puzzlebot_controller/controllers/controller_interface.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        class PurePursuitController : public ControllerInterface {
            private:
                double lookahead_distance_;
            public:
                PurePursuitController(double linear_speed, double lookahead_distance);

                geometry_msgs::msg::Twist computeCommand(
                    const geometry_msgs::msg::PoseStamped& current_pose,
                    const std::vector<geometry_msgs::msg::PoseStamped>& path) override;
        };
    }
}

#endif