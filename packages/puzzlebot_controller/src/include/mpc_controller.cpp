#include "puzzlebot_controller/controllers/mpc_controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

using namespace puzzlebot_controllers::controllers;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MPCController::MPCController(double linear_speed, double horizon, double dt)
  : ControllerInterface(linear_speed), horizon_(horizon), dt_(dt) {
  steps_ = static_cast<int>(horizon_ / dt_);
}

bool MPCController::computeCommand(
  const geometry_msgs::msg::PoseStamped& current_pose,
  const std::vector<geometry_msgs::msg::PoseStamped>& path,
  geometry_msgs::msg::Twist::SharedPtr cmd)
{
  if (path.empty()) return true;

  // Use a simple kinematic model: x_{t+1} = x_t + v*dt*cos(theta), etc.

  // Find nearest path point
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); ++i) {
    double dx = path[i].pose.position.x - current_pose.pose.position.x;
    double dy = path[i].pose.position.y - current_pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // Estimate orientation
  tf2::Quaternion q(
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w
  );
  double theta = tf2::getYaw(q);

  // Initialize state vector: [x, y, theta]
  VectorXd state(3);
  state << current_pose.pose.position.x, current_pose.pose.position.y, theta;

  // Generate a naive trajectory using constant linear speed
  double best_cost = std::numeric_limits<double>::max();
  double best_angular_z = 0.0;

  for (double delta_w = -1.5; delta_w <= 1.5; delta_w += 0.1) {
    VectorXd x = state;
    double cost = 0.0;

    for (int t = 0; t < steps_; ++t) {
      // Apply control
      x(0) += linear_speed_ * std::cos(x(2)) * dt_;
      x(1) += linear_speed_ * std::sin(x(2)) * dt_;
      x(2) += delta_w * dt_;

      // Compare to reference path
      if (closest_idx + t < path.size()) {
        double dx = path[closest_idx + t].pose.position.x - x(0);
        double dy = path[closest_idx + t].pose.position.y - x(1);
        cost += dx * dx + dy * dy;
      }
    }

    if (cost < best_cost) {
      best_cost = cost;
      best_angular_z = delta_w;
    }
  }

  cmd->linear.x = linear_speed_;
  cmd->angular.z = best_angular_z;

  // Stop if at end
  return (closest_idx >= path.size() - 1);
}
