#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "puzzlebot_navigation/include/core_slam.h"

#include <unordered_map>
#include <cmath>
#include <memory>
#include <vector>


class CoreSLAM : public rclcpp::Node {
    