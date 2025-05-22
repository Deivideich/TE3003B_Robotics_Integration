#include "rclcpp/rclcpp.hpp"
#include "puzzlebot_planning/model/se2_state.hpp" // Include the SE2State header
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a minimal node (doesn't need to spin for this test)
  auto node = std::make_shared<rclcpp::Node>("test_state_node");

  RCLCPP_INFO(node->get_logger(), "Test State Node started.");

  // Create an instance of SE2State
  puzzlebot_planning::model::SE2State my_state(1.2345, 6.789, 0.5);

  // Use the toString() method and print it using RCLCPP_INFO
  RCLCPP_INFO(node->get_logger(), "Created State: %s", my_state.toString().c_str());

  // Create another state using the default constructor
  puzzlebot_planning::model::SE2State default_state;
  RCLCPP_INFO(node->get_logger(), "Default State: %s", default_state.toString().c_str());

  // Test setters and getters
  default_state.setX(10.0);
  default_state.setY(-5.0);
  RCLCPP_INFO(node->get_logger(), "Modified Default State: %s", default_state.toString().c_str());
  RCLCPP_INFO(node->get_logger(), "Getter test - X: %.3f", default_state.getX());


  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Test State Node shutting down.");
  return 0;
}
