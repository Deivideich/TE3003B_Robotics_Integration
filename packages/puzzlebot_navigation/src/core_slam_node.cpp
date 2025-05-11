#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "puzzlebot_navigation/slam/core_slam.h"

#include <cmath>
#include <memory>
#include <vector>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // for toMsg/fromMsg
#include <tf2/utils.h> // for getYaw()

using std::placeholders::_1;

class CoreSlamNode : public rclcpp::Node
{
public:
    CoreSlamNode()
    : Node("core_slam_node"), 
      slam_(100, 3, 0.01, 0.01),
      has_scan_(false), first_scan_(true), last_odom_set_(false)
    {
        max_particle = std::make_shared<Particle>();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CoreSlamNode::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CoreSlamNode::odom_callback, this, _1));

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/constructed_map", 10);
        particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particles", 10);

        // Create a timer to periodically broadcast the transform
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CoreSlamNode::broadcast_transform, this));


        RCLCPP_INFO(this->get_logger(), "CoreSLAM Node Initialized. Before TF");
    }

    void init_tf()
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this());

        RCLCPP_INFO(this->get_logger(), "TF Broadcaster and Listener Initialized.");
    }


private:
    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    std::shared_ptr<Particle> max_particle;
    puzzlebot_navigation::SLAM::CoreSLAM slam_;
    std::shared_ptr<std::vector<float>> scan_angles_, scan_ranges_;
    float max_range_;
    int scan_size_;

    bool has_scan_, first_scan_;
    geometry_msgs::msg::Pose last_odom_;
    bool last_odom_set_;

    const float trans_threshold_ = 0.1;  // meters
    const float rot_threshold_ = M_PI/32;    // radians

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_size_ = msg->ranges.size();
        max_range_ = msg->range_max;
        scan_angles_ = std::make_shared<std::vector<float>>(scan_size_);
        scan_ranges_ = std::make_shared<std::vector<float>>(msg->ranges);

        for (int i = 0; i < scan_size_; ++i) {
            (*scan_angles_)[i] = msg->angle_min + i * msg->angle_increment;
        }

        has_scan_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!has_scan_) return;

        const auto& pose = msg->pose.pose;
        float x = pose.position.x;
        float y = pose.position.y;
        float theta = tf2::getYaw(pose.orientation);

        if (first_scan_) {
            slam_.initial_guess(scan_angles_, scan_ranges_, scan_size_, max_range_);
            slam_.updateMapParams();
            first_scan_ = false;
        }

        if (!last_odom_set_) {
            last_odom_ = pose;
            last_odom_set_ = true;
            return;
        }

        float dx = x - last_odom_.position.x;
        float dy = y - last_odom_.position.y;
        float dtheta = theta - tf2::getYaw(last_odom_.orientation);

        float delta_trans = std::sqrt(dx*dx + dy*dy);
        float delta_rot = std::abs(dtheta);

        if (delta_trans > trans_threshold_ || delta_rot > rot_threshold_) {
            slam_.motion_update(dx, dy, dtheta);
            slam_.weight_slam_particles(scan_angles_, scan_ranges_, scan_size_, max_range_, max_particle);
            slam_.resample_particles();
            slam_.updateMapParams();

            publish_particles();
            last_odom_ = pose;
            publish_map();
        }
        
    }

    void publish_particles() {
        auto msg = geometry_msgs::msg::PoseArray();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        const auto& particles = slam_.get_particles();
        for (const auto& [x, y, theta] : *particles) {
            geometry_msgs::msg::Pose p;
            p.position.x = x;
            p.position.y = y;
            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            p.orientation = tf2::toMsg(q);

            msg.poses.push_back(p);
        }

        particle_pub_->publish(msg);
    }

    void broadcast_transform() {
        try {
            if (!max_particle) return;
    
            // Get the best particle's pose in map coordinates
            const auto& [x, y, theta] = *max_particle;
    
            // Get odom -> base_link transform
            geometry_msgs::msg::TransformStamped odom_to_base;
            try {
                odom_to_base = tf_buffer_->lookupTransform(
                    "odom", "base_link", tf2::TimePointZero);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
                return;
            }
    
            auto [map_center_x, map_center_y] = slam_.get_map_center();
    
            // Compute map -> odom transform
            geometry_msgs::msg::TransformStamped map_to_odom;
            map_to_odom.header.stamp = this->get_clock()->now();
            map_to_odom.header.frame_id = "map";
            map_to_odom.child_frame_id = "odom";
    
            // Account for map origin in the transform
            map_to_odom.transform.translation.x = x - map_center_x;
            map_to_odom.transform.translation.y = y - map_center_y;
            map_to_odom.transform.translation.z = 0.0;
    
            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            map_to_odom.transform.rotation = tf2::toMsg(q);
    
            // Compose with odom->base to get proper map->base relationship
            tf_broadcaster_->sendTransform(map_to_odom);
    
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "TF broadcast error: %s", e.what());
        }
    }

    void publish_map() {
        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
    
        auto map_data = slam_.get_main_map();
        auto origin = slam_.get_map_origin();
        auto shape = slam_.get_map_shape();
        float resolution = slam_.get_map_resolution();
    
        msg.info.resolution = resolution;
        msg.info.width = shape[1];
        msg.info.height = shape[0];
        
        // Critical change: Set the origin to the actual map origin
        msg.info.origin.position.x = origin[0];
        msg.info.origin.position.y = origin[1];
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.w = 1.0;  // No rotation
    
        // Initialize map data (-1 = unknown)
        msg.data.assign(shape[0] * shape[1], -1);
    
        // Fill in occupied cells
        for (const auto& [grid_coords, world_coords] : *map_data) {
            int index = grid_coords.first * shape[1] + grid_coords.second;
            if (index >= 0 && index < msg.data.size()) {
                msg.data[index] = 100; // Occupied
            }
        }
    
        map_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoreSlamNode>();
    node->init_tf();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}