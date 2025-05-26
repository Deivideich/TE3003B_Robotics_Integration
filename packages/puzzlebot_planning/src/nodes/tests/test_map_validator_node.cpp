#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h> // Include for ScopedState
#include <memory> // For std::make_shared
#include <visualization_msgs/msg/marker.hpp>
// Include the validator header we created
#include "puzzlebot_planning/state_validity/circle_map_validator.hpp"
#include <chrono>
namespace ob = ompl::base;

using namespace std::chrono_literals;
using std::placeholders::_1;

class MapValidatorTestNode : public rclcpp::Node {
public:
    MapValidatorTestNode() : Node("map_validator_test_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing MapValidatorTestNode...");

        // --- OMPL Setup ---
        state_space_ = std::make_shared<ob::SE2StateSpace>();
        
        // Cast the state space to SE2StateSpace to access setBounds
        auto se2_state_space = std::dynamic_pointer_cast<ob::SE2StateSpace>(state_space_);
        if (se2_state_space) {
            ob::RealVectorBounds bounds(2);
            bounds.setLow(-50);  // Lower bound for x, y
            bounds.setHigh(50); // Upper bound for x, y
            se2_state_space->setBounds(bounds);
        } else {
            RCLCPP_ERROR(this->get_logger(), "State space is not of type SE2StateSpace. Cannot set bounds.");
        }

        space_info_ = std::make_shared<ob::SpaceInformation>(state_space_);

        // --- Validator Setup ---
        double robot_radius = 0.3; // 30 cm
        int occupancy_threshold = 80; // Example threshold (adjust as needed)
        validator_ = std::make_shared<puzzlebot_planning::state_validity::CircleMapValidator>(
            space_info_, robot_radius, occupancy_threshold
        );
        space_info_->setStateValidityChecker(validator_);
        space_info_->setup(); // Finalize setup

        RCLCPP_INFO(this->get_logger(), "Using robot radius: %.2f m, Occupancy threshold: %d",
                    robot_radius, occupancy_threshold);

        // --- ROS Subscriptions ---
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            10, std::bind(&MapValidatorTestNode::mapCallback, this, _1));

        clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&MapValidatorTestNode::clickedPointCallback, this, _1));
        
        // --- Visualization Publisher ---
        circle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/circle_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for map and clicked points...");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received map (Resolution: %.3f, Size: %dx%d)",
                    msg->info.resolution, msg->info.width, msg->info.height);
        latest_map_ = msg;
        // Optionally update validator here, but request was to update on click
    }

    void clickedPointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received clicked point: (%.2f, %.2f) in frame '%s'",
                    msg->point.x, msg->point.y, msg->header.frame_id.c_str());

        if (!latest_map_) {
            RCLCPP_ERROR(this->get_logger(), "No map received yet. Cannot validate point.");
            return;
        }

        // Check if the clicked point frame_id matches the map frame_id
        // Add a check for empty frame_id as well
        if (msg->header.frame_id.empty() || latest_map_->header.frame_id.empty()) {
             RCLCPP_WARN(this->get_logger(), "Map frame ('%s') or clicked point frame ('%s') is empty. Assuming they match, but this is unsafe.",
                        latest_map_->header.frame_id.c_str(), msg->header.frame_id.c_str());
        } else if (msg->header.frame_id != latest_map_->header.frame_id) {
            RCLCPP_ERROR(this->get_logger(), "Clicked point frame ('%s') does not match map frame ('%s'). Cannot validate point without transform.",
                         msg->header.frame_id.c_str(), latest_map_->header.frame_id.c_str());
            return; // Cannot proceed without transform
        }


        // 1. Update the validator with the latest map
        validator_->updateMap(latest_map_);
        RCLCPP_INFO(this->get_logger(), "Validator updated with latest map.");

        // 2. Create an OMPL state representing the clicked point
        // Allocate state using the space information's state space
        ob::ScopedState<ob::SE2StateSpace> ompl_state(space_info_);

        // Set the state values (assuming clicked point is in the map frame)
        ompl_state->setX(msg->point.x);
        ompl_state->setY(msg->point.y);
        ompl_state->setYaw(0.0); // Set a default orientation (e.g., 0 radians)

        // 3. Check validity
        bool is_valid = space_info_->isValid(ompl_state.get()); // Use SpaceInformation's check

        // 4. Log the result
        if (is_valid) {
            RCLCPP_INFO(this->get_logger(), "Clicked point (%.2f, %.2f) is VALID.",
                        ompl_state->getX(), ompl_state->getY());
        } else {
            RCLCPP_WARN(this->get_logger(), "Clicked point (%.2f, %.2f) is INVALID (Collision or out of bounds).",
                       ompl_state->getX(), ompl_state->getY());
        }
        // 5. Visualize the circle around the clicked point
        visualization_msgs::msg::Marker circle_marker;

        circle_marker.header.frame_id = msg->header.frame_id;
        circle_marker.header.stamp = this->now();
        circle_marker.ns = "circle_marker";
        circle_marker.id = 0;
        circle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        circle_marker.action = visualization_msgs::msg::Marker::ADD;
        circle_marker.pose.position.x = msg->point.x;
        circle_marker.pose.position.y = msg->point.y;
        circle_marker.pose.position.z = 0.1; // Assuming 2D plane
        circle_marker.pose.orientation.w = 1.0; // No rotation

        circle_marker.scale.x = 2 * validator_->getRobotRadius(); // Diameter
        circle_marker.scale.y = 2 * validator_->getRobotRadius(); // Diameter
        circle_marker.scale.z = 0.1; // Height of the cylinder
        circle_marker.color.r = is_valid ? 0.0 : 1.0; // Red if invalid
        circle_marker.color.g = is_valid ? 1.0 : 0.0; // Green if valid
        circle_marker.color.b = 0.0; // Blue
        circle_marker.color.a = 0.5; // Transparency
        circle_marker.lifetime = rclcpp::Duration(5000ms); // 5 seconds
        circle_marker_pub_->publish(circle_marker);
        // ScopedState automatically frees the state when it goes out of scope
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    // visualizer for circle size (marker pub)
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr circle_marker_pub_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;

    ob::StateSpacePtr state_space_;
    ob::SpaceInformationPtr space_info_;
    std::shared_ptr<puzzlebot_planning::state_validity::CircleMapValidator> validator_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapValidatorTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
