#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "puzzlebot_interfaces/srv/plan_path.hpp"

#include <future>
#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

template <typename T>
std::future_status wait_for_future_with_timeout(
    typename rclcpp::Client<T>::FutureAndRequestId &future,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::milliseconds timeout = 5s)
{
    auto status = future.wait_for(timeout);
    auto start_time = std::chrono::steady_clock::now();
    while (status != std::future_status::ready) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(10ms);
        status = future.wait_for(10ms);
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time > timeout) {
            return std::future_status::timeout;
        }
    }
    return status;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("astar_planner_client_subscribed");

    auto client = node->create_client<puzzlebot_interfaces::srv::PlanPath>("plan_path");

    geometry_msgs::msg::PoseStamped::SharedPtr goal_pose = nullptr;
    geometry_msgs::msg::PoseStamped::SharedPtr start_pose = nullptr;

    bool done = false;

    auto goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        [&](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            RCLCPP_INFO(node->get_logger(), "Received goal pose");
            goal_pose = msg;
        });

    auto start_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/mcl_pose", 10,
        [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            start_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
            start_pose->header = msg->header;
            start_pose->pose = msg->pose.pose;
            RCLCPP_INFO(node->get_logger(), "Received start pose");
        });

    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !done) {
        rclcpp::spin_some(node);

        if (goal_pose && start_pose) {
            if (!client->wait_for_service(2s)) {
                RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
                break;
            }

            auto request = std::make_shared<puzzlebot_interfaces::srv::PlanPath::Request>();
            request->start = *start_pose;
            request->goal = *goal_pose;

            RCLCPP_INFO(node->get_logger(), "Calling path planner service...");
            auto future = client->async_send_request(request);

            auto status = wait_for_future_with_timeout<puzzlebot_interfaces::srv::PlanPath>(
                future, node->get_node_base_interface(), 5s);

            if (status == std::future_status::timeout) {
                RCLCPP_ERROR(node->get_logger(), "Service call timed out");
            } else if (!future.valid()) {
                RCLCPP_ERROR(node->get_logger(), "Invalid future");
            } else {
                auto response = future.get();
                RCLCPP_INFO(node->get_logger(), "Path planning successful, received %zu points", response->path.size());
                for (const auto &pose : response->path) {
                    float theta = std::atan2(2.0f * (pose.pose.orientation.z * pose.pose.orientation.w), 1.0f - 2.0f * (pose.pose.orientation.z * pose.pose.orientation.z));
                    RCLCPP_INFO(node->get_logger(), "Path point: x=%.2f, y=%.2f, theta=%.2f", pose.pose.position.x, pose.pose.position.y, theta);
                }
            }

            done = true;  // Stop after one execution
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
