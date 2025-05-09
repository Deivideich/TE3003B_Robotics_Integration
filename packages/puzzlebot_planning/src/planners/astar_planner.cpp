#include "puzzlebot_planning/planners/astar_planner.hpp"
#include <algorithm> // For std::min, std::max
#include <cmath> // For std::abs
#include <limits> // For std::numeric_limits
#include <queue>
#include <iostream>
#include <unordered_map>

namespace puzzlebot_planning::planners
{
    AStarPlanner::AStarPlanner(const std::vector<std::vector<int>>& grid, 
                               const float map_resolution,
                               const float map_origin_x,
                               const float map_origin_y,
                               const float theta_resolution,
                               const float translational_weight = 0.5, 
                               const float rotational_weight = 0.5,
                               const int interpolation_steps = 10)
        : grid_(grid), map_resolution_(map_resolution), 
          theta_resolution_(theta_resolution), map_origin_x_(map_origin_x), map_origin_y_(map_origin_y),
          translational_weight_(translational_weight), rotational_weight_(rotational_weight),
          interpolation_steps_(interpolation_steps)
    {
        // Initialize the trajectory object
        trajectory_ = std::make_shared<Trajectory>();
    }

    AStarPlanner::AStarPlanner()
        : translational_weight_(0.5), rotational_weight_(0.5), 
          map_resolution_(1.0), theta_resolution_(M_PI / 8.0), 
          interpolation_steps_(10)
    {
        // Initialize the trajectory object
        trajectory_ = std::make_shared<Trajectory>();
    }

    double AStarPlanner::heuristic(const SE2StatePtr& a, const SE2StatePtr& b) const
    {
        // Euclidean distance heuristic
        double trans = std::sqrt(std::pow(a->getX() - b->getX(), 2) + std::pow(a->getY() - b->getY(), 2));
        double rot_diff = std::abs(a->getTheta() - b->getTheta());
        double rot = std::min(rot_diff, 2 * M_PI - rot_diff); // Ensure the angle difference is within [0, pi]
        return translational_weight_ * trans + rotational_weight_ * rot;
    }

    double AStarPlanner::angleDiff(const SE2StatePtr& a, const SE2StatePtr& b) const
    {
        double diff = std::abs(a->getTheta() - b->getTheta());
        return std::min(diff, 2 * M_PI - diff); // Ensure the angle difference is within [0, pi]
    }

    void AStarPlanner::buildTrajectory(Node* node)
    {
        if (node == nullptr || node->parent == nullptr) return;
        
        if (node->parent){
            buildTrajectory(node->parent); // Recursively build the trajectory
            // Interpolate between parent and current node
            for (int i = 1; i < interpolation_steps_; ++i) {
                double t = static_cast<double>(i) / interpolation_steps_;
                SE2StatePtr interp = std::dynamic_pointer_cast<SE2State>(
                    node->parent->state->interpolate(*node->state, t));
                trajectory_->addState(interp);
            }
        }
        trajectory_->addState(node->state); // Add the current node's state to the trajectory
    }

    bool AStarPlanner::findPath()
    {
        if (grid_.empty() || grid_[0].empty()) {
            std::cerr << "Grid is empty!" << std::endl;
            return false;
        }
        if (start_ == nullptr || goal_ == nullptr) {
            std::cerr << "Start or goal state is not set!" << std::endl;
            return false;
        }
        // A* algorithm implementation
        std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
        std::unordered_map<StateTuple, Node*, hashFunction> closed_set;

        // Initialize the start node
        Node* start_node = new Node();
        start_node->state = start_;
        start_node->g_cost = 0;
        start_node->h_cost = 0;
        start_node->f_cost = 0;
        open_set.push(start_node);

        Node* final_node = new Node(); // Pointer to the final node
        final_node->state = goal_;;
        
        trajectory_->clear(); // Clear the trajectory before starting
        trajectory_->addState(start_); // Add start state to trajectory

        while (!open_set.empty())
        {
            Node* current_node = open_set.top();
            open_set.pop();

            // Check if we reached the goal
            if (current_node->state->distanceTo(*goal_) < 0.1)
            {   
                if (angleDiff(current_node->state, goal_) > M_PI / 16.0){
                    final_node->parent = current_node; // Set the parent of the final node to the current node
                    final_node->g_cost = current_node->g_cost + heuristic(current_node->state, goal_);
                    final_node->h_cost = heuristic(final_node->state, goal_);
                    final_node->f_cost = final_node->g_cost + final_node->h_cost;
                } else {
                    final_node->parent = current_node->parent; // Set the parent of the final node to the parent of the current node
                    final_node->g_cost = current_node->g_cost;
                    final_node->h_cost = heuristic(final_node->state, goal_);
                    final_node->f_cost = final_node->g_cost + final_node->h_cost;
                }


                current_ = final_node; // Set the current node to the goal node
                buildTrajectory(final_node); // Build the trajectory
                return true; // Path found
            }

            int grid_x = static_cast<int>(std::round((current_node->state->getX() - map_origin_x_) / map_resolution_));
            int grid_y = static_cast<int>(std::round((current_node->state->getY() - map_origin_y_) / map_resolution_));
            int theta_bin = static_cast<int>(std::round(current_node->state->getTheta() / theta_resolution_));
            // int theta_bin = static_cast<int>(std::round(current_node->state->getTheta() / theta_resolution_));
            auto it = closed_set.find({grid_y, grid_x, theta_bin});

            if (it != closed_set.end() && it->second->f_cost <= current_node->f_cost)
                        continue; // Skip this neighbor
                    else if (it != closed_set.end())
                        closed_set.erase(it); // Remove from closed set if we found a better path
            // Add current node to closed set
            closed_set[{grid_y, grid_x, theta_bin}] = current_node;

            // Generate neighbors
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    if (dx == 0 && dy == 0) continue; // Skip the current node

                    double new_x = current_node->state->getX() + dx * map_resolution_;
                    double new_y = current_node->state->getY() + dy * map_resolution_;

                    int new_grid_x = static_cast<int>(std::round((new_x - map_origin_x_) / map_resolution_));
                    int new_grid_y = static_cast<int>(std::round((new_y - map_origin_y_) / map_resolution_));

                    // Check if the new position is within bounds and not an obstacle
                    if (new_grid_y < 0 || new_grid_y >= grid_.size() || new_grid_x < 0 || new_grid_x >= grid_[0].size() || grid_[new_grid_y][new_grid_x] == 1)
                        continue;
                    
                    // Calculate the new theta based on the movement direction
                    double new_theta = std::atan2(dy * map_resolution_, dx * map_resolution_);
                    int new_theta_bin = static_cast<int>(std::round(new_theta / theta_resolution_));
                    // Create a new state for the neighbor
                    SE2StatePtr neighbor_state = std::make_shared<SE2State>(new_x, new_y, new_theta);

                    // Calculate costs
                    double g_cost = current_node->g_cost + heuristic(neighbor_state, current_node->state);;
                    double h_cost = heuristic(neighbor_state, goal_);
                    double f_cost = g_cost + h_cost;

                    // Check if the neighbor is already in the closed set
                    auto it = closed_set.find({new_grid_y, new_grid_x, new_theta_bin});
                    if (it != closed_set.end() && it->second->f_cost <= f_cost)
                        continue; // Skip this neighbor
                    else if (it != closed_set.end())
                        closed_set.erase(it); // Remove from closed set if we found a better path
                    
                    Node* neighbor_node = new Node();
                    neighbor_node->state = neighbor_state;
                    neighbor_node->g_cost = g_cost;
                    neighbor_node->h_cost = h_cost;
                    neighbor_node->f_cost = f_cost;
                    neighbor_node->parent = current_node;
                    open_set.push(neighbor_node);
                }
            }
        }
        // If we reach here, no path was found
        trajectory_->clear(); // Clear the trajectory if no path is found

        return false; // Path not found
    }
}
