#ifndef PUZZLEBOT_CONTROLLER__CONTROLLERS__BUG0_CONTROLLER_HPP_
#define PUZZLEBOT_CONTROLLER__CONTROLLERS__BUG0_CONTROLLER_HPP_

#include "puzzlebot_controller/bug_controllers/bug_controller_interface.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <memory>
#include <vector>
#include <unordered_set>



namespace puzzlebot_controllers 
{
    namespace bug_controllers 
    {
        enum RobotState {
            AVOIDING_OBSTACLE,
            NAV_TO_GOAL,
            REACHED,
            IDLE
        };
        
        struct hashFunction
        {
            size_t operator()(const std::pair<int,int> &key) const{
            auto [x, y] = key;
            return x ^ y;
        }
    };

        class Bug0Controller :  public BugControllerInterface {
            public:
                Bug0Controller(double linear_speed, double angular_speed, double desired_distance, double angle_delta, int dfs_scale);
                
                bool computeCommand(
                    const geometry_msgs::msg::PoseStamped& current_pose,
                    const geometry_msgs::msg::PoseStamped& goal_pose,
                    geometry_msgs::msg::Twist::SharedPtr cmd) override;

            private:
                geometry_msgs::msg::Twist driveToPose(const geometry_msgs::msg::PoseStamped& target_pose,
                                                      const geometry_msgs::msg::PoseStamped& current_pose);

                bool isCloseToGoal(const geometry_msgs::msg::PoseStamped& current_pose, const geometry_msgs::msg::PoseStamped& goal_pose);
                bool isLineToGoalBlocked(const geometry_msgs::msg::PoseStamped& current_pose, const geometry_msgs::msg::PoseStamped& goal_pose);
                std::pair<int,int> toGrid(double wx, double wy, const nav_msgs::msg::OccupancyGrid& grid);

                void getDFSObject(const nav_msgs::msg::OccupancyGrid& grid, std::pair<int,int> coord, std::unordered_set<std::pair<int,int>, hashFunction>& visited, std::vector<std::pair<int,int>>& object_corners);
            
                geometry_msgs::msg::PoseStamped::SharedPtr computeNextPose();
                
                RobotState state_;
                std::vector<geometry_msgs::msg::PointStamped>* obstacle_blob_ = nullptr;
                double desired_distance_;
                double angular_speed_;
                double angle_delta_;
                int dfs_scale_;
                std::vector<std::pair<int,int>> directions_ = { {-1,1}, {0,1}, {1,1},
                                                                {-1,0}, {0,0}, {1,0},
                                                                {-1,-1}, {0,-1}, {1,-1} };
        };
    }
}

#endif