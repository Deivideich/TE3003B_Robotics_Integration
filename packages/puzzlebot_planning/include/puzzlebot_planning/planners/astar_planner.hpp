#ifndef PUZZLEBOT_PLANNING_PLANNERS_ASTAR_PLANNER_HPP_
#define PUZZLEBOT_PLANNING_PLANNERS_ASTAR_PLANNER_HPP_


#include "puzzlebot_planning/model/trajectory.hpp"
#include "puzzlebot_planning/model/se2_state.hpp"
#include "puzzlebot_planning/model/state.hpp"
#include "puzzlebot_planning/planners/planner.hpp"
#include <vector>
#include <cmath> // For std::sqrt, std::pow
#include <memory>

typedef std::vector<std::vector<int>> Grid;
typedef std::tuple<int, int, int> StateTuple;

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
            size_t operator()(const StateTuple &key) const{
            auto [x, y, theta] = key;
            return std::hash<int>()(x) ^ std::hash<int>()(y) ^ std::hash<int>()(theta);
        }
    };

    /**
     * @brief A* pathfinding algorithm for 2D grid maps.
     */
    class AStarPlanner : public Planner
    {
    private:
        std::vector<std::vector<int>> grid_; // 2D grid map (0 for free space, 1 for obstacle) // AFTER SAMPLING WONT BE NEEDED
        std::vector<std::pair<float,float>> base_footprint_;
        float map_resolution_;  // Default value
        float map_origin_x_; // Default value
        float map_origin_y_; // Default value
        float theta_resolution_; // Default value
        float translational_weight_; // Weight for translational cost
        float rotational_weight_; // Weight for rotational cost

        SE2StatePtr start_; // Starting state
        SE2StatePtr goal_; // Goal state
        Node* current_ = nullptr; // Current state, always initialized to nullptr
        TrajectoryPtr trajectory_; // Trajectory object to store the path
        int interpolation_steps_;

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
                     const std::vector<std::pair<float,float>>& base_footprint,
                     const float map_resolution,
                     const float map_origin_x,
                     const float map_origin_y,
                     const float theta_resolution,
                     const float translational_weight, 
                     const float rotational_weight,
                     const int interpolation_steps);

        /**
         * @brief Empty constructor for AStarPlanner.
         */
        AStarPlanner();

        /**
         * @brief Check that the state is collision free.
         * @param state The state to check.
         */
        bool isFootPrintCollisionFree(const SE2StatePtr& state) const;

        /**
         * @brief Find the shortest path from start to goal using A* algorithm.
         * @return A vector of states representing the path.
         */
        bool plan() override;

        /**
         * @brief Angle diff between states.
         * @param a The first state.
         * @param b The second state.
         * @return Absolute value of the angle diff.
         */
        double angleDiff(const SE2StatePtr& a, const SE2StatePtr& b) const;
        
        /**
         * @brief Build the trajectory from the current node to the start node.
         * @param current_node The current node.
         */
        void buildTrajectory(Node* current_node);
        
        // Getters
        TrajectoryPtr getTrajectory() const override { return trajectory_; }
        StatePtr getStart() const override { return start_; }
        StatePtr getGoal() const override { return goal_; }
        Node* getCurrentNode() const { return current_; }
        Grid getGrid() const { return grid_; }
        bool isUsingRealSampling() const { return using_real_sampling_; }
        float getMapResolution() const { return map_resolution_; }
        std::pair<float,float> getMapOrigin() const { return std::make_pair(map_origin_x_, map_origin_y_); }
        float getMapOriginX() const { return map_origin_x_; }
        float getMapOriginY() const { return map_origin_y_; }
        std::vector<std::pair<float,float>> getBaseFootprint() const { return base_footprint_; }
        

        // Setters
        void setStart(const StatePtr& start) override { start_ = std::dynamic_pointer_cast<SE2State>(start); }
        void setGoal(const StatePtr& goal) override { goal_ = std::dynamic_pointer_cast<SE2State>(goal); }
        void setCurrentNode(Node* current) { current_ = current; }
        void setGrid(const Grid& grid) { grid_ = grid; }
        void setTrajectory(const TrajectoryPtr& trajectory) { trajectory_ = trajectory; }
        void setUsingRealSampling(bool using_real_sampling) { using_real_sampling_ = using_real_sampling; }
        void setTranslationalWeight(float translational_weight) { translational_weight_ = translational_weight; }
        void setRotationalWeight(float rotational_weight) { rotational_weight_ = rotational_weight; }
    };
}

#endif