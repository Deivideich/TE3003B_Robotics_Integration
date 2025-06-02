#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "puzzlebot_controller/controllers/controller_interface.hpp"
#include "puzzlebot_controller/controllers/pure_pursuit.hpp"
#include "puzzlebot_controller/controllers/pid_controller.hpp"
#include "puzzlebot_controller/controllers/bug2_controller.hpp"
// #include "mpc_controller.hpp"
#include "puzzlebot_interfaces/srv/plan_path.hpp"


#include <future>
#include <memory>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include <unordered_map>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    // Declare params
    controller_type_ = this->declare_parameter<std::string>("controller_type", "pure_pursuit");
    linear_speed_ = this->declare_parameter<float>("linear_speed", 0.1);
    lookahead_distance_ = this->declare_parameter<float>("lookahead_distance", 0.2);
    kP_ = this->declare_parameter<float>("kP", 0.2);
    kI_ = this->declare_parameter<float>("kI", 0.2);
    kD_ = this->declare_parameter<float>("kD", 0.2);
    usingBugAlgorithm_ = this->declare_parameter<bool>("usingBugAlgorithm", false);
    usingMCLPose_ = this->declare_parameter<bool>("usingMCLPose", true);
  }

  void get_parameters(){
    controller_type_ = this->get_parameter("controller_type").as_string();
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    kP_ = this->get_parameter("kP").as_double();
    kI_ = this->get_parameter("kI").as_double();
    kD_ = this->get_parameter("kD").as_double();
    usingBugAlgorithm_ = this->get_parameter("usingBugAlgorithm").as_bool();
    usingMCLPose_ = this->get_parameter("usingMCLPose").as_bool();
  }

  void setup(){
    get_parameters();

    if (controller_type_ == "pure_pursuit") {
      controller_ = std::make_unique<puzzlebot_controllers::controllers::PurePursuitController>(linear_speed_, lookahead_distance_);
    } else if (controller_type_ == "pid") {
      controller_ = std::make_unique<puzzlebot_controllers::controllers::PIDController>(linear_speed_, kP_, kD_, kI_);
    // } else if (controller_type_ == "mpc") {
      // controller_ = std::make_unique<puzzlebot_controllers::controllers::MPCController>(linear_speed_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown controller type: %s", controller_type_.c_str());
      rclcpp::shutdown();
    }

    if (usingBugAlgorithm_){
      bug_controller_ = std::make_unique<puzzlebot_controllers::controllers::Bug2Controller>(linear_speed_);
    }

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    planner_client_ = this->create_client<puzzlebot_interfaces::srv::PlanPath>("plan_path", rmw_qos_profile_services_default, client_cb_group_);

    curr_pose_listener_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topics[usingMCLPose_], 10, std::bind(&ControllerNode::poseCallback, this, _1)); 
    goal_listener_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&ControllerNode::goalCallback, this, _1)); 
    local_map_listener_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/local_map", 10, std::bind(&ControllerNode::localMapCallback, this, _1));

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

  void localMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    local_map_ = msg;

    if (usingBugAlgorithm_) bug_controller_->setLocalMap(msg);
  }

  void activateBug2Mode() {
    RCLCPP_WARN(this->get_logger(), "Activating Bug-2 mode.");
    geometry_msgs::msg::Point current = current_pose_->pose.position;
    geometry_msgs::msg::Point goal = goal_pose_->pose.position;
    bug_controller_->reset();
    bug_controller_->setMLine(current, goal);
    bug_mode_active_ = true;
  }

  void needsReplanning(){
    if (bug_mode_active_ && usingBugAlgorithm_) {          
      geometry_msgs::msg::Twist::SharedPtr cmd = std::make_shared<geometry_msgs::msg::Twist>();
      if(bug_controller_->computeCommand(*current_pose_, current_path_, cmd)){
        RCLCPP_INFO(this->get_logger(), "Bug mode done, resuming path tracking.");
        bug_controller_->reset();  // Typo fixed: from 'resut' to 'reset'
        needs_planning_ = true;
        bug_mode_active_ = false;
      }

      cmd_pub_->publish(*cmd);
    } else if (usingBugAlgorithm_ && controller_->getPathIndex() + 1 < current_path_.size() && 
        bug_controller_->isDirectionBlocked(*current_pose_, -(M_PI / 8), (M_PI / 8), (M_PI / 16), 0.05, true)) {
      activateBug2Mode();
    } 

    // CHECK IF NEEDS PLANNING DEPENDING ON CONTROLLER_GETPATHINDEX POSE AND CURRENT POSE 
    if (!current_path_.empty() && controller_->getPathIndex() < current_path_.size()) {
      const auto& target_pose = current_path_[controller_->getPathIndex()].pose;
      const auto& current_position = current_pose_->pose.position;

      double dx = target_pose.position.x - current_position.x;
      double dy = target_pose.position.y - current_position.y;
      double distance_to_target = std::sqrt(dx * dx + dy * dy);
      // RCLCPP_INFO(this->get_logger(), "Distance: %2.2f", distance_to_target)
      if (distance_to_target > lookahead_distance_ * 2) {
        RCLCPP_WARN(this->get_logger(), "Significant deviation detected. Replanning required.");
        needs_planning_ = true;
      }
    }
  }

  void timerCallback(){
    if (!current_pose_ || !goal_pose_) return;
    
    if (needs_planning_) {
      needs_planning_ = false;

      auto request = std::make_shared<puzzlebot_interfaces::srv::PlanPath::Request>();
      request->start = *current_pose_;
      request->goal = *goal_pose_;

      auto future_result = planner_client_->async_send_request(request);
      // Set up a callback for when the future is complete
      while (future_result.wait_for(100ms) != std::future_status::ready);

      auto response = future_result.get();

      if (response->result){
        current_path_ = response->path;
        controller_->resetIndex();
      } else {
        RCLCPP_WARN(this->get_logger(), "Could not find a path");
        goal_pose_ = nullptr;
      }
    }

    needsReplanning();

    if (!current_path_.empty() && !bug_mode_active_ && ! needs_planning_) {
      geometry_msgs::msg::Twist::SharedPtr cmd = std::make_shared<geometry_msgs::msg::Twist>();
      if (controller_->computeCommand(*current_pose_, current_path_, cmd)) {
        RCLCPP_INFO(this->get_logger(), "Achieved goal!");

        goal_pose_ = nullptr;
        needs_planning_ = true;
      }

      cmd_pub_->publish(*cmd);
    }    
  }

  // TODO: function that checkes current error with supposed path pose and current pose
  bool needsNewPlanning() { return needs_planning_;}

  std::string controller_type_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr curr_pose_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<puzzlebot_interfaces::srv::PlanPath>::SharedPtr planner_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  
  std::vector<geometry_msgs::msg::PoseStamped> current_path_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ = nullptr;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_ = nullptr;
  nav_msgs::msg::OccupancyGrid::SharedPtr local_map_ = nullptr;

  bool needs_planning_ = true;
  bool bug_mode_active_ = false;
  bool usingBugAlgorithm_;
  bool usingMCLPose_;
  std::unordered_map<bool,std::string> pose_topics = {{false, "/kalman_pose"}, {true, "/mcl_pose"}};

  std::unique_ptr<puzzlebot_controllers::controllers::ControllerInterface> controller_;
  std::unique_ptr<puzzlebot_controllers::controllers::Bug2Controller> bug_controller_;
  double linear_speed_;
  double lookahead_distance_;
  double kP_, kD_, kI_;
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

