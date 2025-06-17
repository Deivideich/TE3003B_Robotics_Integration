#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "puzzlebot_interfaces/action/controller_action.hpp"

using namespace std::chrono_literals;
using ControllerAction = puzzlebot_interfaces::action::ControllerAction;

class GoalPoseClient : public rclcpp::Node
{
public:
  GoalPoseClient() : Node("goal_pose_client")
  {
    action_client_ = rclcpp_action::create_client<ControllerAction>(this, "controller_server");

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&GoalPoseClient::goal_pose_callback, this, std::placeholders::_1));

    stop_keyboard_thread_ = false;
    keyboard_thread_ = std::thread(&GoalPoseClient::keyboard_listener, this);
  }

  ~GoalPoseClient() override
  {
    stop_keyboard_thread_ = true;
    if (keyboard_thread_.joinable())
      keyboard_thread_.join();
  }

private:
  rclcpp_action::Client<ControllerAction>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp_action::ClientGoalHandle<ControllerAction>::SharedPtr current_goal_handle_;
  std::thread keyboard_thread_;
  std::atomic<bool> stop_keyboard_thread_;

  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!action_client_->wait_for_action_server(2s))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    ControllerAction::Goal goal_msg;
    goal_msg.goal = *msg;

    RCLCPP_INFO(this->get_logger(), "Sending goal to controller action...");

    auto send_goal_options = rclcpp_action::Client<ControllerAction>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<ControllerAction>::SharedPtr,
             const std::shared_ptr<const ControllerAction::Feedback> feedback)
    {
      RCLCPP_INFO(this->get_logger(), "Received feedback: %s",
                  feedback->controller_state.c_str());
    };

    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<ControllerAction>::WrappedResult & result)
    {
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Goal was canceled");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
      }
    };

    auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);

    std::thread([this, future_goal_handle]() mutable {
      auto goal_handle = future_goal_handle.get();
      current_goal_handle_ = goal_handle;
    }).detach();
  }

  void keyboard_listener()
  {
    while (!stop_keyboard_thread_)
    {
      char c = std::cin.get();
      if (c == 's' || c == 'S')
      {
        if (current_goal_handle_)
        {
          RCLCPP_INFO(this->get_logger(), "Canceling goal due to 's' key press...");
          action_client_->async_cancel_goal(current_goal_handle_);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
        }
      }
      std::this_thread::sleep_for(100ms);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPoseClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}