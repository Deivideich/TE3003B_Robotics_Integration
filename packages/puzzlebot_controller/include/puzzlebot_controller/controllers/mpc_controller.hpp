#ifndef PUZZLEBOT_CONTROLLERS_MPC_CONTROLLER_HPP
#define PUZZLEBOT_CONTROLLERS_MPC_CONTROLLER_HPP

#include "puzzlebot_controller/controllers/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>

namespace puzzlebot_controllers {
namespace controllers {

class MPCController : public ControllerInterface {
public:
  MPCController(double linear_speed, double horizon, double dt);

  bool computeCommand(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const std::vector<geometry_msgs::msg::PoseStamped>& path,
    geometry_msgs::msg::Twist::SharedPtr cmd) override;

private:
  double horizon_;
  double dt_;
  int steps_;
};

}  // namespace controllers
}  // namespace puzzlebot_controllers

#endif  // PUZZLEBOT_CONTROLLERS_MPC_CONTROLLER_HPP