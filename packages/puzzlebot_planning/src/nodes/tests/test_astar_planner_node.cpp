#include "rclcpp/rclcpp.hpp"
#include "puzzlebot_planning/model/se2_state.hpp"
#include "puzzlebot_planning/model/trajectory.hpp"
#include "puzzlebot_planning/planners/astar_planner.hpp"
#include <vector>
#include <memory> // For std::make_shared

using namespace puzzlebot_planning::model;
using namespace puzzlebot_planning::planners;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_astar_planner_node");

    RCLCPP_INFO(node->get_logger(), "Starting A* Planner test...");

    // Define a simple grid (0 for free space, 1 for obstacle)
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 0, 1, 0}
    };
    std::vector<std::pair<float,float>> base_footprint = {{0.0,0.0},{0.0,0.0}}; // x min y min, x max y max

    // Create start and goal states
    auto start_state = std::make_shared<SE2State>(0.0, 0.0, 0.0);
    auto goal_state = std::make_shared<SE2State>(40.0, 40.0, M_PI / 2.0);

    // Create an A* planner instance
    AStarPlanner astar_planner(grid, base_footprint, 10.0, 0,0, M_PI / 8, 0.5, 0.5, 100); // Weights for translational and rotational costs

    // Set start and goal states
    astar_planner.setStart(start_state);
    astar_planner.setGoal(goal_state);

    // Find the path
    bool result = astar_planner.findPath();

    if (result){
        // Print the resulting trajectory
        RCLCPP_INFO(node->get_logger(), "Generated Trajectory:\n%s", astar_planner.getTrajectory()->toString().c_str());

        // Node* current_node = astar_planner.getCurrentNode();
        // std::vector<SE2StatePtr> path;
        // while (current_node != nullptr) {
        //     path.push_back(current_node->state);
        //     current_node = current_node->parent;
        // }

        // std::reverse(path.begin(), path.end()); // Reverse the path to get it from start to goal

        // std::vector<std::vector<char>> visualization(grid.size(), std::vector<char>(grid[0].size(), '.'));

        // // Mark obstacles
        // for (size_t i = 0; i < grid.size(); ++i) {
        //     for (size_t j = 0; j < grid[i].size(); ++j) {
        //     if (grid[i][j] == 1) {
        //         visualization[i][j] = '#';
        //     }
        //     }
        // }

        // // Mark the path
        // for (const auto& state : path) {
        //     int x = static_cast<int>(state->getX());
        //     int y = static_cast<int>(state->getY());
        //     if (x >= 0 && x < static_cast<int>(grid.size()) && y >= 0 && y < static_cast<int>(grid[0].size())) {
        //     visualization[x][y] = '*';
        //     }
        // }

        // // Mark start and goal
        // visualization[static_cast<int>(start_state->getX())][static_cast<int>(start_state->getY())] = 'S';
        // visualization[static_cast<int>(goal_state->getX())][static_cast<int>(goal_state->getY())] = 'G';

        // // Print the visualization
        // RCLCPP_INFO(node->get_logger(), "Path visualization:");
        // for (const auto& row : visualization) {
        //     std::string line(row.begin(), row.end());
        //     RCLCPP_INFO(node->get_logger(), "%s", line.c_str());
        // }
    }
    else {
        RCLCPP_ERROR(node->get_logger(), "No path found.");
    }

    RCLCPP_INFO(node->get_logger(), "A* Planner test finished.");

    rclcpp::shutdown();
    return 0;
}