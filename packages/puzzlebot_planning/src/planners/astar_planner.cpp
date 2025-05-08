#include "puzzlebot_planning/planners/astar_planner.hpp"
#include <algorithm> // For std::min, std::max
#include <cmath> // For std::abs
#include <limits> // For std::numeric_limits
#include <queue>
#include <iostream>

namespace puzzlebot_planning::planners
{
    AStarPlanner::AStarPlanner(const std::vector<std::vector<int>>& grid, 
                               const float translational_weight, 
                               const float rotational_weight)
        : grid_(grid), translational_weight_(translational_weight), rotational_weight_(rotational_weight)
    {
        // Initialize the trajectory object
        trajectory_ = std::make_shared<Trajectory>();
    }

    AStarPlanner::AStarPlanner()
        : translational_weight_(0.5), rotational_weight_(0.5)
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

    bool AStarPlanner::isGoalReached(const SE2StatePtr& state) const
    {
        if (using_real_sampling_)
        {
            // Check if the current state is close to the goal state
            return state->distanceTo(*goal_) < 0.1; // Adjust the threshold as needed
        } else  {
            return heuristic(state, goal_) < 0.1; // Adjust the threshold as needed
        }
    }

    void AStarPlanner::buildTrajectory(Node* node)
    {
        if (node == nullptr || node->parent == nullptr) return;
        buildTrajectory(node->parent); // Recursively build the trajectory
        trajectory_->addState(node->state); // Add the current node's state to the trajectory
    }

    bool AStarPlanner::findPath()
    {
        // A* algorithm implementation
        std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
        std::unordered_map<std::pair<int , int>, Node*, hashFunction> closed_set;

        // Initialize the start node
        Node* start_node = new Node();
        start_node->state = start_;
        start_node->g_cost = 0;
        start_node->h_cost = 0;
        start_node->f_cost = 0;
        open_set.push(start_node);

        trajectory_->addState(start_); // Add start state to trajectory

        while (!open_set.empty())
        {
            Node* current_node = open_set.top();
            open_set.pop();

            // Check if we reached the goal
            if (current_node->state->distanceTo(*goal_) < 0.1)
            {
                std::cout << "Path found!" << std::endl;
                current_ = current_node; // Set the current node to the goal node
                buildTrajectory(current_node); // Build the trajectory
                return true; // Path found
            }

            // Add current node to closed set
            closed_set[{static_cast<int>(current_node->state->getX()), static_cast<int>(current_node->state->getY())}] = current_node;

            // Generate neighbors
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    if (dx == 0 && dy == 0) continue; // Skip the current node

                    double new_x = current_node->state->getX() + dx;
                    double new_y = current_node->state->getY() + dy;

                    // Check if the new position is within bounds and not an obstacle
                    if (new_x < 0 || new_x >= grid_.size() || new_y < 0 || new_y >= grid_[0].size() || grid_[static_cast<int>(new_x)][static_cast<int>(new_y)] == 1)
                        continue;
                    
                    // Calculate the new theta based on the movement direction
                    double new_theta = std::atan2(dy, dx);
                    // Create a new state for the neighbor
                    SE2StatePtr neighbor_state = std::make_shared<SE2State>(new_x, new_y, new_theta);

                    // Calculate costs
                    double g_cost = current_node->g_cost + heuristic(neighbor_state, current_node->state);;
                    double h_cost = heuristic(neighbor_state, goal_);
                    double f_cost = g_cost + h_cost;

                    // Check if the neighbor is already in the closed set
                    auto it = closed_set.find({static_cast
                        <int>(new_x), static_cast<int>(new_y)});
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
