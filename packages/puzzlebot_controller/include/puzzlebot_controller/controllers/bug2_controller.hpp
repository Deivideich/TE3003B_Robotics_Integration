#ifndef PUZZLEBOT_CONTROLLER__CONTROLLERS__BUG2_CONTROLLER_HPP_
#define PUZZLEBOT_CONTROLLER__CONTROLLERS__BUG2_CONTROLLER_HPP_

// #include "puzzlebot_controller/controllers/controller_interface.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <memory>


namespace puzzlebot_controllers 
{
    namespace controllers 
    {
        enum RobotState {
            TURNING_LEFT,
            FORWARD,
            TURNING_RIGHT,
            IDLE
        };

        class Bug2Controller {
            public:
                Bug2Controller(double linear_speed);
                
                bool computeCommand(
                    const geometry_msgs::msg::PoseStamped& current_pose,
                    const std::vector<geometry_msgs::msg::PoseStamped>& path,
                    geometry_msgs::msg::Twist::SharedPtr cmd);
                
                void reset();

                bool isDirectionBlocked(const geometry_msgs::msg::PoseStamped& pose, 
                    double angle_min, double angle_max, double angle_step, 
                    double distance);

                void setMLine(const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& goal);
                void setLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr local_map) {local_map_ = local_map;}
            private:
                bool onMLine(const geometry_msgs::msg::Point& current_pos);
                bool closerToGoal(const geometry_msgs::msg::Point& current_pos);
                    
                geometry_msgs::msg::Point mline_start_, mline_goal_;
                geometry_msgs::msg::Point hit_point_;

                double min_distance_to_goal_;
                double linear_speed_;
                double desired_angle_;
                bool circling_obstacle_ = false;
                bool left_mline_ = false;

                RobotState state_;

                nav_msgs::msg::OccupancyGrid::SharedPtr local_map_ = nullptr;
        };
    }
}

#endif