#ifndef PUZZLEBOT_CONTROLLER__CONTROLLERS__PURE_PURSUIT_HPP_
#define PUZZLEBOT_CONTROLLER__CONTROLLERS__PURE_PURSUIT_HPP_

#include "puzzlebot_controller/controllers/controller_interface.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <memory>
#include <angles/angles.h>

namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        class PurePursuitController : public ControllerInterface {
            private:
                double lookahead_distance_;
                double orientation_tolerance_;
            public:
                PurePursuitController(double linear_speed, double angular_speed, double lookahead_distance, double orientation_tolerance);

                bool computeCommand(
                    const geometry_msgs::msg::PoseStamped& current_pose,
                    const std::vector<geometry_msgs::msg::PoseStamped>& path,
                    geometry_msgs::msg::Twist::SharedPtr cmd) override;
        };
    }
}

#endif