#ifndef PUZZLEBOT_PLANNING_PLANNERS_ASTAR_PLANNER_HPP_
#define PUZZLEBOT_PLANNING_PLANNERS_ASTAR_PLANNER_HPP_


#include "puzzlebot_planning/model/trajectory.hpp"
#include "puzzlebot_planning/model/se2_state.hpp"
#include "puzzlebot_planning/model/state.hpp"
#include <vector>
#include <cmath> // For std::sqrt, std::pow
#include <memory>

typedef std::vector<std::vector<int>> Grid;

using namespace puzzlebot_planning::model;

namespace puzzlebot_planning::planners
{ 
    struct Node
    {
        Node* parent = nullptr; // Pointer to parent node
        SE2StatePtr state;
        double g_cost; // Cost from start to this node
        double h_cost; // Heuristic cost to goal
        double f_cost; // Total cost (g + h)
    };

    class NodeCompare
    {   
        public:
            bool operator()(const Node* a, const Node* b) const
            {
                return a->f_cost > b->f_cost; // Min-heap based on f_cost
            }
    };

    struct hashFunction
        {
            size_t operator()(const std::pair<int , int> &x) const{
            return x.first ^ x.second;
        }
    };

    /**
     * @brief A* pathfinding algorithm for 2D grid maps.
     */
    class AStarPlanner
    {
    private:
        std::vector<std::vector<int>> grid_; // 2D grid map (0 for free space, 1 for obstacle) // AFTER SAMPLING WONT BE NEEDED
        SE2StatePtr start_; // Starting state
        Node* current_ = nullptr; // Current state, always initialized to nullptr
        SE2StatePtr goal_; // Goal state
        float translational_weight_; // Weight for translational cost
        float rotational_weight_; // Weight for rotational cost
        TrajectoryPtr trajectory_; // Trajectory object to store the path
        bool using_real_sampling_ = false; // Flag to indicate if real sampling is used

        /**
         * @brief Heuristic function for A* algorithm (Euclidean distance).
         * @param a The first state.
         * @param b The second state.
         * @return The heuristic cost.
         */
        double heuristic(const SE2StatePtr& a, const SE2StatePtr& b) const;
    public:
        /**
         * @brief Construct a new AStarPlanner object.
         * @param grid The 2D grid map (0 for free space, 1 for obstacle).
         * @param translational_weight The translational weight.
         * @param rotational_weight The rotational weight.
         */
        AStarPlanner(const std::vector<std::vector<int>>& grid, 
                     const float translational_weight, 
                     const float rotational_weight);

        /**
         * @brief Empty constructor for AStarPlanner.
         */
        AStarPlanner();

        /**
         * @brief Check if the goal state is reached.
         * @param state The current state.
         * @return True if the goal is reached, false otherwise.
         */
        bool isGoalReached(const SE2StatePtr& state) const;

        /**
         * @brief Find the shortest path from start to goal using A* algorithm.
         * @return A vector of states representing the path.
         */
        bool findPath();
        
        /**
         * @brief Build the trajectory from the current node to the start node.
         * @param current_node The current node.
         */
        void buildTrajectory(Node* current_node);
        
        // Getters
        StatePtr getStart() const { return start_; }
        Node* getCurrentNode() const { return current_; }
        StatePtr getGoal() const { return goal_; }
        Grid getGrid() const { return grid_; }
        TrajectoryPtr getTrajectory() const { return trajectory_; }
        bool isUsingRealSampling() const { return using_real_sampling_; }

        // Setters
        void setStart(const SE2StatePtr& start) { start_ = start; }
        void setCurrentNode(Node* current) { current_ = current; }
        void setGoal(const SE2StatePtr& goal) { goal_ = goal; }
        void setGrid(const Grid& grid) { grid_ = grid; }
        void setTrajectory(const TrajectoryPtr& trajectory) { trajectory_ = trajectory; }
        void setUsingRealSampling(bool using_real_sampling) { using_real_sampling_ = using_real_sampling; }
        void setTranslationalWeight(float translational_weight) { translational_weight_ = translational_weight; }
        void setRotationalWeight(float rotational_weight) { rotational_weight_ = rotational_weight; }
    };
}

#endif