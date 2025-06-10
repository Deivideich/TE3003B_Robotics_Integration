#ifndef PUZZLEBOT_CONTROLLER__BUG_CONTROLLERS__BUG_CONTROLLER_INTERFACE_HPP_
#define PUZZLEBOT_CONTROLLER__BUG_CONTROLLERS__BUG_CONTROLLER_INTERFACE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <memory>

namespace puzzlebot_controllers 
{
    namespace bug_controllers 
    {
        class BugControllerInterface {
            private:
                nav_msgs::msg::OccupancyGrid::SharedPtr local_map_ = nullptr;
                geometry_msgs::msg::PoseStamped::SharedPtr goal_ = nullptr;
                geometry_msgs::msg::PoseStamped control_pose_;

                double linear_speed_;
                int path_index_ = 0;
            public:
                BugControllerInterface(double linear_speed) { linear_speed_ = linear_speed; }

                virtual ~BugControllerInterface() = default;
                
                virtual bool computeCommand(
                    const geometry_msgs::msg::PoseStamped& current_pose,
                    const geometry_msgs::msg::PoseStamped& goal_pose,
                    geometry_msgs::msg::Twist::SharedPtr cmd
                ) = 0;
                
                void reset() { setPathIndex(0); }
                void updateMap(nav_msgs::msg::OccupancyGrid::SharedPtr local_map) { local_map_ = local_map; }
                
                // SETTER AND GETTERS
                // void setGoalPose(geometry_msgs::msg::PoseStamped goal) { goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(goal); } 
                void setControlPose(geometry_msgs::msg::PoseStamped control_pose) { control_pose_ = control_pose; }
                double getLinearSpeed() { return linear_speed_; }
                nav_msgs::msg::OccupancyGrid::SharedPtr getLocalMap() { return local_map_; }
                // geometry_msgs::msg::PoseStamped getGoalPose() { return *goal_; }
                geometry_msgs::msg::PoseStamped getControlPose() { return control_pose_; }
                void setPathIndex(int new_index) { path_index_ = new_index; } 
                int getPathIndex() const { return path_index_; }
        };
    }
}


#endif