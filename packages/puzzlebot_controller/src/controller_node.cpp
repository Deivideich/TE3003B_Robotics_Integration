#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "puzzlebot_controller/controllers/controller_interface.hpp"
#include "puzzlebot_controller/controllers/pure_pursuit.hpp"
#include "puzzlebot_controller/controllers/pid_controller.hpp"
// #include "mpc_controller.hpp"
#include "puzzlebot_interfaces/srv/plan_path.hpp"


#include <future>
#include <memory>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>  // ✅ This is the key one

using std::placeholders::_1;
using namespace std::chrono_literals;

enum ControllerStates{
  STOPPED,
  GLOBAL_PLANNING,
  BUG2_PLANNING,
  OBSTACLE_FOUND,
  GLOBAL_CONTROLLER,
  LOCAL_CONTROLLER,
};

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    // Declare params
    controller_type_ = this->declare_parameter<std::string>("controller_type", "pure_pursuit");
    usingBugAlgorithm_ = this->declare_parameter<bool>("usingBugAlgorithm", true);
    delta_angle_ = this->declare_parameter<float>("delta_angle", float(M_PI / 32));
    deviation_threshold_ = this->declare_parameter<float>("deviation_threshold", 0.75);
    
    linear_speed_ = this->declare_parameter<float>("linear_speed", 0.1);
    lookahead_distance_ = this->declare_parameter<float>("lookahead_distance", 0.2);
    kP_ = this->declare_parameter<float>("kP", 0.2);
    kI_ = this->declare_parameter<float>("kI", 0.2);
    kD_ = this->declare_parameter<float>("kD", 0.2);
  }

  void get_parameters(){
    controller_type_ = this->get_parameter("controller_type").as_string();
    usingBugAlgorithm_ = this->get_parameter("usingBugAlgorithm").as_bool();
    delta_angle_ = this->get_parameter("delta_angle").as_double();
    deviation_threshold_ = this->get_parameter("deviation_threshold").as_double();

    linear_speed_ = this->get_parameter("linear_speed").as_double();
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    kP_ = this->get_parameter("kP").as_double();
    kI_ = this->get_parameter("kI").as_double();
    kD_ = this->get_parameter("kD").as_double();
  }

  void setup(){
    get_parameters();

    if (controller_type_ == "pure_pursuit") {
      controller_ = std::make_unique<puzzlebot_controllers::controllers::PurePursuitController>(linear_speed_, lookahead_distance_);
      if (usingBugAlgorithm_){
      /**
       * Previously we used a wall following algorithm with a bug2 controller, however we will know implement a "BUG2Planner" and the BUG2 controller will work
       * as a pure pursuit controller, we will create a plan that follows the bug theory
       */
      bug_controller_ = std::make_unique<puzzlebot_controllers::controllers::PurePursuitController>(linear_speed_, lookahead_distance_);
    }
    } else if (controller_type_ == "pid") {
      controller_ = std::make_unique<puzzlebot_controllers::controllers::PIDController>(linear_speed_, kP_, kD_, kI_);
      if (usingBugAlgorithm_){
      /**
       * Previously we used a wall following algorithm with a bug2 controller, however we will know implement a "BUG2Planner" and the BUG2 controller will work
       * as a pure pursuit controller, we will create a plan that follows the bug theory
       */
      bug_controller_ = std::make_unique<puzzlebot_controllers::controllers::PIDController>(linear_speed_,  kP_, kD_, kI_);
      }
    // } else if (controller_type_ == "mpc") {
      // controller_ = std::make_unique<puzzlebot_controllers::controllers::MPCController>(linear_speed_);
      //bug_controller_ = std::make_unique<puzzlebot_controllers::controllers::MPCController>(linear_speed_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown controller type: %s", controller_type_.c_str());
      rclcpp::shutdown();
    }

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    planner_client_ = this->create_client<puzzlebot_interfaces::srv::PlanPath>("plan_path", rmw_qos_profile_services_default, client_cb_group_);
    bug_planner_client_ = this->create_client<puzzlebot_interfaces::srv::PlanPath>("bug_plan_path", rmw_qos_profile_services_default, client_cb_group_);

    curr_pose_listener_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/mcl_pose", 10, std::bind(&ControllerNode::poseCallback, this, _1)); 
    goal_listener_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&ControllerNode::goalCallback, this, _1)); 
    // local_map_listener_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/local_map", 10, std::bind(&ControllerNode::localMapCallback, this, _1));
    merged_map_listener_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/merged_map", 10, std::bind(&ControllerNode::mergedMapCallback, this, _1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&ControllerNode::timerCallback, this), timer_cb_group_);

    RCLCPP_INFO(this->get_logger(), "Waiting for planning service");
    while (!planner_client_->wait_for_service(2s));
    RCLCPP_INFO(this->get_logger(), "Planner server ready");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    current_pose_->header = msg->header;
    current_pose_->pose = msg->pose.pose;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    if (goal_pose_ != nullptr) {
      RCLCPP_INFO(this->get_logger(), "Received **NEW** goal pose: [x: %f, y: %f, z: %f, orientation: (%f, %f, %f, %f)]",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      needs_planning_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Received goal pose: [x: %f, y: %f, z: %f, orientation: (%f, %f, %f, %f)]",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    }
    goal_pose_ = msg;
  }

  void mergedMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    merged_map_ = msg;
  }

  // void localMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  //   local_map_ = msg;

  //   if (usingBugAlgorithm_) bug_controller_->setLocalMap(msg);
  // }

  double angle_diff(double a, double b){
    double diff = a - b;
    while (diff > M_PI) diff -= 2*M_PI;
    while (diff < -M_PI) diff += 2*M_PI;
    
    return diff;
  }

  bool isPathBlocked(const geometry_msgs::msg::PoseStamped& curr_pose, 
                          const geometry_msgs::msg::PoseStamped& desired_pose,
                          double angle_delta) 
  {
      if (!merged_map_) return false;

      // Get robot's orientation (yaw)
      tf2::Quaternion q(
          curr_pose.pose.orientation.x,
          curr_pose.pose.orientation.y,
          curr_pose.pose.orientation.z,
          curr_pose.pose.orientation.w
      );
      double robot_angle = tf2::getYaw(q);

      // Get orientations (yaw) of current and desired poses
      tf2::Quaternion q_desired(
          desired_pose.pose.orientation.x,
          desired_pose.pose.orientation.y,
          desired_pose.pose.orientation.z,
          desired_pose.pose.orientation.w
      );
      double desired_angle = tf2::getYaw(q_desired);

      // Calculate check_distance as the distance between curr_pose and desired_pose
      double dx = desired_pose.pose.position.x - curr_pose.pose.position.x;
      double dy = desired_pose.pose.position.y - curr_pose.pose.position.y;
      double check_distance = std::hypot(dx, dy);

      // Use the average of robot_angle and desired_angle as the center, and half their difference as the angle_delta
      double center_angle = (robot_angle + desired_angle) / 2.0;
      double angle_span = std::abs(angle_diff(desired_angle, robot_angle));
      double min_range = center_angle - angle_span / 2.0;
      double max_range = center_angle + angle_span / 2.0;
      int num_rays = 15; // Number of rays to cast within the angle range

      auto worldToMap = [this](double x, double y, int& mx, int& my) {
          mx = static_cast<int>((x - merged_map_->info.origin.position.x) / merged_map_->info.resolution);
          my = static_cast<int>((y - merged_map_->info.origin.position.y) / merged_map_->info.resolution);
      };

      int width = merged_map_->info.width;
      int height = merged_map_->info.height;

      geometry_msgs::msg::Point start = curr_pose.pose.position;

      for (int i = 0; i < num_rays; ++i) {
          double angle = min_range + i * (max_range - min_range) / (num_rays - 1);
          double end_x = start.x + check_distance * std::cos(angle);
          double end_y = start.y + check_distance * std::sin(angle);

          int x0, y0, x1, y1;
          worldToMap(start.x, start.y, x0, y0);
          worldToMap(end_x, end_y, x1, y1);

          // Bresenham's line algorithm
          int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
          int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
          int err = dx + dy, e2;

          int cx = x0, cy = y0;
          while (true) {
              if (cx < 0 || cx >= width || cy < 0 || cy >= height) break;
              int idx = cy * width + cx;
              if (merged_map_->data[idx] > 50) // Occupied threshold
                  return true;

              if (cx == x1 && cy == y1) break;
              e2 = 2 * err;
              if (e2 >= dy) { err += dy; cx += sx; }
              if (e2 <= dx) { err += dx; cy += sy; }
          }
      }
      return false;
  }

  // void activateBug2Mode() {
  //   RCLCPP_WARN(this->get_logger(), "Activating Bug-2 mode.");
  //   geometry_msgs::msg::Point current = current_pose_->pose.position;
  //   geometry_msgs::msg::Point goal = goal_pose_->pose.position;
  //   bug_controller_->reset();
  //   bug_controller_->setMLine(current, goal);
  //   bug_mode_active_ = true;
  // }

  // void needsReplanning(){
  //   if (bug_mode_active_ && usingBugAlgorithm_) {          
  //     geometry_msgs::msg::Twist::SharedPtr cmd = std::make_shared<geometry_msgs::msg::Twist>();
  //     if(bug_controller_->computeCommand(*current_pose_, current_path_, cmd)){
  //       RCLCPP_INFO(this->get_logger(), "Bug mode done, resuming path tracking.");
  //       bug_controller_->reset();  // Typo fixed: from 'resut' to 'reset'
  //       needs_planning_ = true;
  //       bug_mode_active_ = false;
  //     }

  //     cmd_pub_->publish(*cmd);
  //   } else if (usingBugAlgorithm_ && controller_->getPathIndex() + 1 < current_path_.size() && 
  //       bug_controller_->isDirectionBlocked(*current_pose_, -(M_PI / 8), (M_PI / 8), (M_PI / 16), 0.05, true)) {
  //     activateBug2Mode();
  //   } 

  //   // CHECK IF NEEDS PLANNING DEPENDING ON CONTROLLER_GETPATHINDEX POSE AND CURRENT POSE 
  //   if (!current_path_.empty() && controller_->getPathIndex() < current_path_.size()) {
  //     const auto& target_pose = current_path_[controller_->getPathIndex()].pose;
  //     const auto& current_position = current_pose_->pose.position;

  //     double dx = target_pose.position.x - current_position.x;
  //     double dy = target_pose.position.y - current_position.y;
  //     double distance_to_target = std::sqrt(dx * dx + dy * dy);
  //     // RCLCPP_INFO(this->get_logger(), "Distance: %2.2f", distance_to_target)
  //     if (distance_to_target > lookahead_distance_ * 2) {
  //       RCLCPP_WARN(this->get_logger(), "Significant deviation detected. Replanning required.");
  //       needs_planning_ = true;
  //     }
  //   }
  // }

  void timerCallback(){
    geometry_msgs::msg::Twist::SharedPtr cmd = std::make_shared<geometry_msgs::msg::Twist>();

    switch (controller_state_)
    {
      case ControllerStates::STOPPED:
      {
        if (current_pose_ != nullptr && goal_pose_ != nullptr) {
          controller_state_ = GLOBAL_PLANNING;
        }
      }
        break;
      case ControllerStates::GLOBAL_PLANNING:
      {
        const auto& goal_pose = *goal_pose_;
        const auto& current_pose = *current_pose_;
        
        auto request = std::make_shared<puzzlebot_interfaces::srv::PlanPath::Request>();
        request->start = current_pose;
        request->goal = goal_pose;

        auto future_result = planner_client_->async_send_request(request);
        // Set up a callback for when the future is complete
        while (future_result.wait_for(100ms) != std::future_status::ready);

        auto response = future_result.get();

        if (response->result){
          current_path_ = response->path;
          controller_state_ = GLOBAL_CONTROLLER;
          RCLCPP_INFO(this->get_logger(), "Succesfully found a path");
        } else {
          RCLCPP_WARN(this->get_logger(), "Could not find a path");
          goal_pose_ = nullptr;
          controller_state_ = STOPPED;
        }
        controller_->resetIndex();
      }
        break;
      case ControllerStates::GLOBAL_CONTROLLER:
      {
        const auto& target_pose = current_path_[controller_->getPathIndex()];
        const auto& current_pose = *current_pose_;

        double dx = target_pose.pose.position.x - current_pose.pose.position.x;
        double dy = target_pose.pose.position.y - current_pose.pose.position.y;
        double distance_to_target = std::sqrt(dx * dx + dy * dy);
        
        if (distance_to_target > deviation_threshold_) {
          RCLCPP_WARN(this->get_logger(), "Significant deviation detected. Replanning required.");
          controller_state_ = GLOBAL_PLANNING;
        }else if (isPathBlocked(current_pose, target_pose, delta_angle_)){
          RCLCPP_WARN(this->get_logger(), "Path is blocked.");
          controller_state_ = OBSTACLE_FOUND;
        } else if (controller_->computeCommand(current_pose, current_path_, cmd)) {
          RCLCPP_INFO(this->get_logger(), "Achieved goal!");

          goal_pose_ = nullptr;
          controller_state_ = STOPPED;
        } 
      }
        break;
      case ControllerStates::OBSTACLE_FOUND:
      {
        if (usingBugAlgorithm_){
          RCLCPP_INFO(this->get_logger(), "Obstacle found using BUG2 planner");
          controller_state_ = BUG2_PLANNING;
        } else {
          RCLCPP_INFO(this->get_logger(), "Obstacle found but not using bug algorithm, passing into replan from current pose");
          controller_state_ = GLOBAL_PLANNING;
        }
      }
        break;
      case ControllerStates::BUG2_PLANNING:
      {
        const auto& target_pose = current_path_[controller_->getPathIndex()];
        const auto& current_pose = *current_pose_;

        auto request = std::make_shared<puzzlebot_interfaces::srv::PlanPath::Request>();
        request->start = current_pose;
        request->goal = target_pose;

        auto future_result = bug_planner_client_->async_send_request(request);
        // Set up a callback for when the future is complete
        while (future_result.wait_for(100ms) != std::future_status::ready);

        auto response = future_result.get();

        if (response->result){
          bug_current_path_ = response->path;
          controller_state_ = LOCAL_CONTROLLER;
        } else {
          RCLCPP_WARN(this->get_logger(), "Could not find a path using BUG2 controller, switching to Global Planning");
          controller_state_ = GLOBAL_PLANNING;
        }
        bug_controller_->resetIndex();
      }
        break;
      case ControllerStates::LOCAL_CONTROLLER:
      {
        const auto& target_pose = bug_current_path_[bug_controller_->getPathIndex()];
        const auto& current_pose = *current_pose_;

        // Check if there is obstacle within path
        if (isPathBlocked(current_pose, target_pose, delta_angle_)){
          RCLCPP_WARN(this->get_logger(), "BUG2 Path is blocked.");
          controller_state_ = OBSTACLE_FOUND;
        } else if (bug_controller_->computeCommand(current_pose, bug_current_path_, cmd)) {
          RCLCPP_INFO(this->get_logger(), "Achieved better position with BUG2 algorithm, switching to global planning since obstacle has being avoided!");
          controller_state_ = GLOBAL_PLANNING;
        }
      }
        break;
      default:
        break;
    }

    cmd_pub_->publish(*cmd);
    
    // if (needs_planning_) {
    //   needs_planning_ = false;

    //   auto request = std::make_shared<puzzlebot_interfaces::srv::PlanPath::Request>();
    //   request->start = *current_pose_;
    //   request->goal = *goal_pose_;

    //   auto future_result = planner_client_->async_send_request(request);
    //   // Set up a callback for when the future is complete
    //   while (future_result.wait_for(100ms) != std::future_status::ready);

    //   auto response = future_result.get();

    //   if (response->result){
    //     current_path_ = response->path;
    //     controller_->resetIndex();
    //   } else {
    //     RCLCPP_WARN(this->get_logger(), "Could not find a path");
    //     goal_pose_ = nullptr;
    //   }
    // }

    // needsReplanning();

    // if (!current_path_.empty() && !bug_mode_active_ && ! needs_planning_) {
    //   geometry_msgs::msg::Twist::SharedPtr cmd = std::make_shared<geometry_msgs::msg::Twist>();
    //   if (controller_->computeCommand(*current_pose_, current_path_, cmd)) {
    //     RCLCPP_INFO(this->get_logger(), "Achieved goal!");

    //     goal_pose_ = nullptr;
    //     needs_planning_ = true;
    //   }

    //   cmd_pub_->publish(*cmd);
    // }    
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr curr_pose_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_listener_;
  // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<puzzlebot_interfaces::srv::PlanPath>::SharedPtr planner_client_;
  rclcpp::Client<puzzlebot_interfaces::srv::PlanPath>::SharedPtr bug_planner_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ = nullptr;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_ = nullptr;
  
  nav_msgs::msg::OccupancyGrid::SharedPtr local_map_ = nullptr;
  nav_msgs::msg::OccupancyGrid::SharedPtr merged_map_ = nullptr;

  bool needs_planning_ = true;
  bool bug_mode_active_ = false;

  // Node Params
  std::string controller_type_;
  bool usingBugAlgorithm_;
  double delta_angle_;
  double deviation_threshold_;

  double linear_speed_;
  double lookahead_distance_;
  double kP_, kD_, kI_;

  std::unique_ptr<puzzlebot_controllers::controllers::ControllerInterface> controller_;
  std::vector<geometry_msgs::msg::PoseStamped> current_path_;
  std::unique_ptr<puzzlebot_controllers::controllers::ControllerInterface> bug_controller_;
  std::vector<geometry_msgs::msg::PoseStamped> bug_current_path_;
  
  ControllerStates controller_state_ = STOPPED;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ControllerNode>();
  node->setup();  // this initializes the controller and subscriptions

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

