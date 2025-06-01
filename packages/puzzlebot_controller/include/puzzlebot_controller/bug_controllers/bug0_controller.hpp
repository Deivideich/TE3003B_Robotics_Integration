#ifndef PUZZLEBOT_CONTROLLER__CONTROLLERS__BUG0_CONTROLLER_HPP_
#define PUZZLEBOT_CONTROLLER__CONTROLLERS__BUG0_CONTROLLER_HPP_

#include "puzzlebot_controller/bug_controllers/bug_controller_interface.hpp"
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
    namespace bug_controllers 
    {
        enum RobotState {
            AVOIDING_OBSTACLE,
            NAV_TO_GOAL,
            REACHED,
            IDLE
        };

        class Bug0Controller :  public BugControllerInterface {
            public:
                Bug0Controller(double linear_speed, double angular_speed, double desired_distance);
                
                bool computeCommand(
                    const geometry_msgs::msg::PoseStamped& current_pose,
                    geometry_msgs::msg::Twist::SharedPtr cmd) override;

                geometry_msgs::msg::PoseStamped transfromToBaselink(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
                       
            private:
                geometry_msgs::msg::Twist driveToPose(const geometry_msgs::msg::PoseStamped& target_pose,
                                                                      const geometry_msgs::msg::PoseStamped& current_pose);

                bool isCloseToGoal(const geometry_msgs::msg::PoseStamped& current_pose);
                bool isLineToGoalBLocked(const geometry_msgs::msg::PoseStamped& current_pose);
                
                geometry_msgs::msg::Point findClosestObstacleInCone(double angle_min, double angle_max);
                geometry_msgs::msg::PoseStamped computeNextPose();

                RobotState state_;
                geometry_msgs::msg::PoseStamped::SharedPtr obstacle_pose;
                double desired_distance_;
                double angular_speed_;
        };
    }
}

#endif