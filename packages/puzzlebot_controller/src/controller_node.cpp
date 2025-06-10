#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include "puzzlebot_interfaces/srv/plan_path.hpp"
#include "puzzlebot_interfaces/action/controller_action.hpp"

#include "puzzlebot_controller/controllers/controller_interface.hpp"
#include "puzzlebot_controller/controllers/pure_pursuit.hpp"
#include "puzzlebot_controller/controllers/pid_controller.hpp"
// #include "mpc_controller.hpp"

#include "puzzlebot_controller/bug_controllers/bug_controller_interface.hpp"
#include "puzzlebot_controller/bug_controllers/bug0_controller.hpp"
// #include "puzzlebot_controller/bug_controllers/bug1_controller.hpp"
// #include "puzzlebot_controller/bug_controllers/bug2_controller.hpp"

#include <future>
#include <memory>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // for tf2::doTransform
#include <tf2/utils.h>  // ✅ This is the key one
#include <unordered_map>

using std::placeholders::_1;
using namespace std::chrono_literals;
using ControllerAction = puzzlebot_interfaces::action::ControllerAction;
using GoalHandleController = rclcpp_action::ServerGoalHandle<ControllerAction>;


enum ControllerStates{
  STOPPED,
  GLOBAL_PLANNING,
  GLOBAL_CONTROLLER,
  OBSTACLE_FOUND,
  BUG2_PLANNING,
  BUG_CONTROLLER,
};

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    // Declare params
    controller_type_ = this->declare_parameter<std::string>("controller_type", "pure_pursuit");
    usingBugAlgorithm_ = this->declare_parameter<bool>("usingBugAlgorithm", true);
    usingMCLPose_ = this->declare_parameter<bool>("usingMCLPose", true);
    delta_angle_ = this->declare_parameter<float>("delta_angle", float(M_PI / 32));
    deviation_threshold_ = this->declare_parameter<float>("deviation_threshold", 0.75);
    
    linear_speed_ = this->declare_parameter<float>("linear_speed", 0.1);
    angular_speed_ = this->declare_parameter<float>("angular_speed", 0.25);
    lookahead_distance_ = this->declare_parameter<float>("lookahead_distance", 0.2);
    orientation_tolerance_ = this->declare_parameter<float>("orientation_tolerance", 0.15);
    kP_ = this->declare_parameter<float>("kP", 0.2);
    kI_ = this->declare_parameter<float>("kI", 0.2);
    kD_ = this->declare_parameter<float>("kD", 0.2);
  }

  void get_parameters(){
    controller_type_ = this->get_parameter("controller_type").as_string();
    usingBugAlgorithm_ = this->get_parameter("usingBugAlgorithm").as_bool();
    usingMCLPose_ = this->get_parameter("usingMCLPose").as_bool();
    delta_angle_ = this->get_parameter("delta_angle").as_double();
    deviation_threshold_ = this->get_parameter("deviation_threshold").as_double();

    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    orientation_tolerance_ = this->get_parameter("orientation_tolerance").as_double();
    kP_ = this->get_parameter("kP").as_double();
    kI_ = this->get_parameter("kI").as_double();
    kD_ = this->get_parameter("kD").as_double();
  }

  void setup(){
    get_parameters();


    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    planner_client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    controller_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    action_server_ = rclcpp_action::create_server<ControllerAction>(
      this,
      "controller_server",
      std::bind(&ControllerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ControllerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&ControllerNode::handleAccepted, this, std::placeholders::_1)
    );

    if (controller_type_ == "pure_pursuit") {
      controller_ = std::make_unique<puzzlebot_controllers::controllers::PurePursuitController>(linear_speed_, angular_speed_, lookahead_distance_, orientation_tolerance_, tf_buffer_.get());
    } else if (controller_type_ == "pid") {
      controller_ = std::make_unique<puzzlebot_controllers::controllers::PIDController>(linear_speed_, angular_speed_, kP_, kD_, kI_, tf_buffer_.get());
    // } else if (controller_type_ == "mpc") {
      // controller_ = std::make_unique<puzzlebot_controllers::controllers::MPCController>(linear_speed_, angular_speed_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown controller type: %s", controller_type_.c_str());
      rclcpp::shutdown();
    }

    if (usingBugAlgorithm_){
      bug_controller_ = std::make_unique<puzzlebot_controllers::bug_controllers::Bug0Controller>(linear_speed_, 0.1, 0.5, delta_angle_, 2);
    }

    planner_client_ = this->create_client<puzzlebot_interfaces::srv::PlanPath>("plan_path", rmw_qos_profile_services_default, planner_client_cb_group_);
    // bug_planner_client_ = this->create_client<puzzlebot_interfaces::srv::PlanPath>("bug_plan_path", rmw_qos_profile_services_default, planner_client_cb_group_);

    curr_pose_listener_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topics[usingMCLPose_], 10, std::bind(&ControllerNode::poseCallback, this, _1)); 
    // goal_listener_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&ControllerNode::goalCallback, this, _1)); 
    local_map_listener_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/local_map", 10, std::bind(&ControllerNode::localMapCallback, this, _1));
    merged_map_listener_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/merged_map", 10, std::bind(&ControllerNode::mergedMapCallback, this, _1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    control_state_pub_ = this->create_publisher<std_msgs::msg::String>("/controller_state", 10);
    control_point_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/control_point", 10);
    // timer_ = this->create_wall_timer(100ms, std::bind(&ControllerNode::controllerFSM, this), timer_cb_group_);
    // timer_cb_group_

    RCLCPP_INFO(this->get_logger(), "Waiting for planning service");
    while (!planner_client_->wait_for_service(2s));
    RCLCPP_INFO(this->get_logger(), "Planner server ready");
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ControllerAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    ignore_obstacles_ = goal->ignore_obstacles;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleController> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleController> goal_handle)
  {
    current_goal_handle_ = goal_handle;
    std::thread{std::bind(&ControllerNode::executeGoal, this, goal_handle)}.detach();
  }

  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    current_pose_->header = msg->header;
    current_pose_->pose = msg->pose.pose;
  }

  // void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
  //   if (goal_pose_ != nullptr) {
  //     RCLCPP_INFO(this->get_logger(), "Received **NEW** goal pose: [x: %f, y: %f, z: %f, orientation: (%f, %f, %f, %f)]",
  //           msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
  //           msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  //   } else {
  //     RCLCPP_INFO(this->get_logger(), "Received goal pose: [x: %f, y: %f, z: %f, orientation: (%f, %f, %f, %f)]",
  //           msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
  //           msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  //   }
  //   goal_pose_ = msg;
  // }

  void mergedMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    merged_map_ = msg;
  }

  void localMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    local_map_ = msg;
    if (usingBugAlgorithm_) bug_controller_->updateMap(local_map_);
  }

  geometry_msgs::msg::PoseStamped transfromToBaselink(const geometry_msgs::msg::PoseStamped::SharedPtr pose, bool overwriteStamp = false){
    geometry_msgs::msg::PoseStamped result;
    if (!pose) {
      RCLCPP_WARN(this->get_logger(), "Input pose is null");
      return result;
    } else if(overwriteStamp){
      pose->header.stamp = local_map_->header.stamp;
    }

    try {
      // Wait for transform to be available
      std::string target_frame = "base_link";
      if (!tf_buffer_->canTransform(target_frame, pose->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1))) {
        RCLCPP_WARN(this->get_logger(), "Transform from %s to %s not available", pose->header.frame_id.c_str(), target_frame.c_str());
        return result;
      }
      tf_buffer_->transform(*pose, result, target_frame, tf2::durationFromSec(1));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    }
    return result;
  }                      

  double angle_diff(double a, double b){
    double diff = a - b;
    while (diff > M_PI) diff -= 2*M_PI;
    while (diff < -M_PI) diff += 2*M_PI;
    
    return diff;
  }

  bool isPathBlocked (const geometry_msgs::msg::PoseStamped& curr_pose, 
                   const geometry_msgs::msg::PoseStamped& desired_pose,
                   double angle_delta) 
  {
      if (!local_map_) return false;
      
      // Compute direction and check distance in base_link frame
      double dx = desired_pose.pose.position.x;
      double dy = desired_pose.pose.position.y;
      double desired_angle = std::atan2(dy, dx);
      double check_distance = std::hypot(dx, dy);

      double min_range = desired_angle - angle_delta;
      double max_range = desired_angle + angle_delta;
      int num_rays = 15;

      int width = local_map_->info.width;
      int height = local_map_->info.height;
      double resolution = local_map_->info.resolution;

      // Robot assumed at center of local map
      int x0 = width / 2;
      int y0 = height / 2;

      for (int i = 0; i < num_rays; ++i) {
          double angle = min_range + i * (max_range - min_range) / (num_rays - 1);
          double end_x = check_distance * std::cos(angle);
          double end_y = check_distance * std::sin(angle);

          int x1 = static_cast<int>(x0 + end_x / resolution);
          int y1 = static_cast<int>(y0 + end_y / resolution);

          // Bresenham's algorithm
          int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
          int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
          int err = dx + dy, e2;

          int cx = x0, cy = y0;
          while (true) {
              if (cx < 0 || cx >= width || cy < 0 || cy >= height) break;
              int idx = cy * width + cx;
              if (local_map_->data[idx] > 50) // Occupied threshold
                  return true;

              if (cx == x1 && cy == y1) break;
              e2 = 2 * err;
              if (e2 >= dy) { err += dy; cx += sx; }
              if (e2 <= dx) { err += dx; cy += sy; }
          }
      }

      return false;
  }

  std_msgs::msg::String stateToString(ControllerStates state) {
    auto message = std_msgs::msg::String();
    switch (state) {
      case ControllerStates::STOPPED: 
        message.data = "STOPPED";
        break;
      case ControllerStates::GLOBAL_PLANNING: 
        message.data = "GLOBAL_PLANNING";
        break;
      case ControllerStates::GLOBAL_CONTROLLER: 
        message.data = "GLOBAL_CONTROLLER";
        break;
      case ControllerStates::OBSTACLE_FOUND: 
        message.data = "OBSTACLE_FOUND";
        break;
      case ControllerStates::BUG2_PLANNING: 
        message.data = "BUG2_PLANNING";
        break;
      case ControllerStates::BUG_CONTROLLER: 
        message.data = "BUG_CONTROLLER";
        break;
      default: 
        message.data = "UNKNOWN";
        break;
    }

    return message;
  }

  void executeGoal(const std::shared_ptr<GoalHandleController> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    goal_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(goal->goal);

    rclcpp::Rate loop_rate(10);
    controller_state_ = GLOBAL_PLANNING;

    auto feedback = std::make_shared<ControllerAction::Feedback>();
    while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Goal canceled");
      goal_handle->canceled(std::make_shared<ControllerAction::Result>());
      controller_state_ = STOPPED;
      goal_pose_ = nullptr;
      break;
    }

    controllerFSM();
    feedback->controller_state = stateToString(controller_state_).data;
    goal_handle->publish_feedback(feedback);

    // If goal is done (e.g., reached):
    if (controller_state_ == STOPPED) {
      auto result = std::make_shared<ControllerAction::Result>();
      result->success = true;
      goal_handle->succeed(result);
      return;
    }

    loop_rate.sleep();
    }

    controllerFSM();
  }

  void controllerFSM(){
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
        }else if (!ignore_obstacles_ && isPathBlocked(transfromToBaselink(current_pose_), transfromToBaselink(std::make_shared<geometry_msgs::msg::PoseStamped>(target_pose), true), delta_angle_)){
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
          RCLCPP_INFO(this->get_logger(), "Obstacle found using BUG2 controller");
          controller_state_ = BUG_CONTROLLER;
        } else {
          RCLCPP_INFO(this->get_logger(), "Obstacle found but not using bug algorithm, passing into replan from current pose");
          controller_state_ = GLOBAL_PLANNING;
        }
      }
        break;
      case ControllerStates::BUG_CONTROLLER:
      {
        const auto& goal_pose = transfromToBaselink(goal_pose_, true);
        const auto& current_pose = transfromToBaselink(current_pose_, true);

        if (bug_controller_->computeCommand(current_pose, goal_pose, cmd)) {
          RCLCPP_INFO(this->get_logger(), "Achieved better position with BUG2 algorithm, switching to global planning since obstacle has being avoided!");
          controller_state_ = GLOBAL_PLANNING;
        } else {
          auto control_pose = bug_controller_->getControlPose();
          control_point_pub_->publish(control_pose);
        }
      }
        break;
      default:
        break;
    }

    cmd_pub_->publish(*cmd);
    control_state_pub_->publish(stateToString(controller_state_));
  }

  rclcpp_action::Server<ControllerAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleController> current_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr curr_pose_listener_;
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr control_point_pub_;
  rclcpp::Client<puzzlebot_interfaces::srv::PlanPath>::SharedPtr planner_client_;
  // rclcpp::Client<puzzlebot_interfaces::srv::PlanPath>::SharedPtr bug_planner_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr planner_client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr controller_cb_group_;
  // rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  
  
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ = nullptr;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_ = nullptr;
  
  nav_msgs::msg::OccupancyGrid::SharedPtr local_map_ = nullptr;
  nav_msgs::msg::OccupancyGrid::SharedPtr merged_map_ = nullptr;

  // TF buffer and tf listener 
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;

  // Node Params
  std::string controller_type_;
  bool usingBugAlgorithm_;
  bool usingMCLPose_;
  double delta_angle_;
  double deviation_threshold_;
  
  std::unordered_map<bool,std::string> pose_topics = {{false, "/ekf_pose"}, {true, "/mcl_pose"}};

  double linear_speed_, angular_speed_;
  double lookahead_distance_, orientation_tolerance_;

  double kP_, kD_, kI_;

  std::unique_ptr<puzzlebot_controllers::controllers::ControllerInterface> controller_;
  std::vector<geometry_msgs::msg::PoseStamped> current_path_;
  std::unique_ptr<puzzlebot_controllers::bug_controllers::BugControllerInterface> bug_controller_;
  // std::vector<geometry_msgs::msg::PoseStamped> bug_current_path_;
  
  ControllerStates controller_state_ = STOPPED;
  bool ignore_obstacles_ = false;  // If true, the controller will ignore obstacles
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

