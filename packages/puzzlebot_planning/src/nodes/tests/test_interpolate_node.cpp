#include "rclcpp/rclcpp.hpp"
#include "puzzlebot_planning/model/se2_state.hpp"
#include "puzzlebot_planning/model/trajectory.hpp"
#include <memory> // For std::make_shared

using namespace puzzlebot_planning::model;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_interpolate_node");

    RCLCPP_INFO(node->get_logger(), "Starting interpolation test...");

    // Define start and goal states
    auto start_state = std::make_shared<SE2State>(0.0, 0.0, 0.0);
    auto goal_state = std::make_shared<SE2State>(5.0, 2.0, M_PI / 4.0);

    RCLCPP_INFO(node->get_logger(), "Start State: %s", start_state->toString().c_str());
    RCLCPP_INFO(node->get_logger(), "Goal State: %s", goal_state->toString().c_str());

    // Create a trajectory
    Trajectory trajectory;
    trajectory.addState(start_state); // Add start state

    // Interpolate between start and goal
    int num_steps = 10;
    for (int i = 1; i < num_steps; ++i) {
        double t = static_cast<double>(i) / num_steps;
        try {
            StatePtr interpolated_state = start_state->interpolate(*goal_state, t);
            trajectory.addState(interpolated_state);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Interpolation error: %s", e.what());
            rclcpp::shutdown();
            return 1;
        }
    }

    trajectory.addState(goal_state); // Add goal state

    // Print the resulting trajectory
    RCLCPP_INFO(node->get_logger(), "Generated Trajectory:\n%s", trajectory.toString().c_str());

    RCLCPP_INFO(node->get_logger(), "Interpolation test finished.");

    rclcpp::shutdown();
    return 0;
}
