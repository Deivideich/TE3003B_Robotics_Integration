#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
    kD_ = this->declare_parameter<float>("kI", 0.2);
    kI_ = this->declare_parameter<float>("kD", 0.2);
  }

  void get_parameters(){
    controller_type_ = this->get_parameter("controller_type").as_string();
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    kP_ = this->get_parameter("kP").as_double();
    kD_ = this->get_parameter("kI").as_double();
    kI_ = this->get_parameter("kD").as_double();
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

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    planner_client_ = this->create_client<puzzlebot_interfaces::srv::PlanPath>("plan_path", rmw_qos_profile_services_default, client_cb_group_);

    curr_pose_listener_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/mcl_pose", 10, std::bind(&ControllerNode::poseCallback, this, _1)); 
    
    goal_listener_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&ControllerNode::goalCallback, this, _1)); 

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&ControllerNode::timerCallback, this), timer_cb_group_);

    RCLCPP_INFO(this->get_logger(), "Waiting for planning service");
    if (!planner_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
    }
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

  void timerCallback(){
    if (!current_pose_ || !goal_pose_) return;
    
    if (needs_planning_) {
      needs_planning_ = false;

      auto request = std::make_shared<puzzlebot_interfaces::srv::PlanPath::Request>();
      request->start = *current_pose_;
      request->goal = *goal_pose_;

      auto future_result = planner_client_->async_send_request(request);
      // Set up a callback for when the future is complete
      future_result.wait_for(250ms);
      if (future_result.valid() && future_result.wait_for(0s) == std::future_status::ready) {
        auto response = future_result.get();
        current_path_ = response->path;
        controller_->resetIndex();
      } else {
        RCLCPP_WARN(this->get_logger(), "Service call timed out or not ready yet");
        goal_pose_ = nullptr;
      }
    }

    if (!current_path_.empty()) {
      auto cmd = controller_->computeCommand(*current_pose_, current_path_);
      cmd_pub_->publish(cmd);

      if (controller_->getPathIndex() == current_path_.size() - 1) {
        RCLCPP_INFO(this->get_logger(), "Achieved goal!");

        goal_pose_ = nullptr;
        needs_planning_ = true;
      }
    }    
  }

  // TODO: function that checkes current error with supposed path pose and current pose
  bool needsNewPlanning() { return needs_planning_;}

  std::string controller_type_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr curr_pose_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<puzzlebot_interfaces::srv::PlanPath>::SharedPtr planner_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  
  std::vector<geometry_msgs::msg::PoseStamped> current_path_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ = nullptr;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_ = nullptr;

  bool needs_planning_ = true;

  std::unique_ptr<puzzlebot_controllers::controllers::ControllerInterface> controller_;
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

