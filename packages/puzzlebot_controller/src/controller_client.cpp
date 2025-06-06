#include <memory>
#include <chrono>

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
    // Create the action client
    action_client_ = rclcpp_action::create_client<ControllerAction>(this, "controller_server");

    // Subscribe to /goal_pose topic
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&GoalPoseClient::goal_pose_callback, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Client<ControllerAction>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!action_client_->wait_for_action_server(2s))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    // Create goal
    ControllerAction::Goal goal_msg;
    goal_msg.goal = *msg;

    RCLCPP_INFO(this->get_logger(), "Sending goal to controller action...");

    // Send goal asynchronously
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

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPoseClient>());
  rclcpp::shutdown();
  return 0;
}
