#include "puzzlebot_planning/planners/ompl_planner.hpp"
#include "puzzlebot_interfaces/srv/plan_path.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OMPLPlannerServer : public rclcpp::Node {
public:
    OMPLPlannerServer() : Node("ompl_planner_server") {
        RCLCPP_INFO(this->get_logger(), "Initializing OMPL Planner Server...");
        
        // Declare and get robot_radius parameter
        this->declare_parameter<double>("robot_radius", 0.1);
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        
        // Declare and get turning_radius parameter
        this->declare_parameter<double>("turning_radius", 30);
        turning_radius_ = this->get_parameter("turning_radius").as_double();
        
        planner_ = std::make_shared<puzzlebot_planning::planners::OMPLPlanner>(robot_radius_, turning_radius_);
        RCLCPP_INFO(this->get_logger(), "OMPL Planner initialized with robot radius: %.2f m, turning radius: %.2f m, Occupancy threshold: %d",
                    robot_radius_, turning_radius_, 20);
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map",
            10,
            std::bind(&OMPLPlannerServer::mapCallback, this, std::placeholders::_1));

        // debugger to check if the map is received
        

        service_ = this->create_service<puzzlebot_interfaces::srv::PlanPath>(
            "plan_path",
            std::bind(&OMPLPlannerServer::handlePlanRequest, this, std::placeholders::_1, std::placeholders::_2));
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&OMPLPlannerServer::clickedPointCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /clicked_point topic.");
        RCLCPP_INFO(this->get_logger(), "OMPL Planner Server is ready.");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        if (planning) {
            return;
        }
        planner_->updateMap(msg);
        RCLCPP_INFO(this->get_logger(), "Map updated.");
    }



    void handlePlanRequest(const std::shared_ptr<puzzlebot_interfaces::srv::PlanPath::Request> request,
                           std::shared_ptr<puzzlebot_interfaces::srv::PlanPath::Response> response) {
        // Set start and goal states
        double yaw, pitch, roll;       

        // Convert quaternion to roll, pitch, yaw START POSE
        geometry_msgs::msg::Quaternion start_quat = request->start.pose.orientation;
        tf2::Quaternion tf_start(start_quat.x, start_quat.y, start_quat.z, start_quat.w); 
        tf2::Matrix3x3(tf_start).getRPY(roll, pitch, yaw);
        
        auto start = std::make_shared<puzzlebot_planning::model::SE2State>(
            request->start.pose.position.x,
            request->start.pose.position.y,
            yaw);

        geometry_msgs::msg::Quaternion goal_quat = request->goal.pose.orientation;
        tf2::Quaternion tf_goal(goal_quat.x, goal_quat.y, goal_quat.z, goal_quat.w);
        tf2::Matrix3x3(tf_goal).getRPY(roll, pitch, yaw);

        auto goal = std::make_shared<puzzlebot_planning::model::SE2State>(
            request->goal.pose.position.x,
            request->goal.pose.position.y,
            yaw);
        
        planner_->setPlanner("RRTSharp");
        planner_->setStart(start);
        planner_->setGoal(goal);
        
        planning = true;
        if (planner_->plan()) {
            planning = false;
            // Get the trajectory
            auto trajectory = planner_->getTrajectory();
            response->result = true;
            response->path.resize(trajectory->getStates().size());

            for (size_t i = 0; i < trajectory->getStates().size(); ++i) {
                const auto& state = std::dynamic_pointer_cast<SE2State>(trajectory->getStates()[i]);
                response->path[i].header.frame_id = "map";
                response->path[i].header.stamp = this->now();
                response->path[i].pose.position.x = state->getX();
                response->path[i].pose.position.y = state->getY();
                response->path[i].pose.orientation.z = std::sin(state->getTheta() / 2);
                response->path[i].pose.orientation.w = std::cos(state->getTheta() / 2);
            }

            // Publish the path
            auto path_msg = nav_msgs::msg::Path();
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = this->now();
            path_msg.poses = response->path;
            path_publisher_->publish(path_msg);
            RCLCPP_INFO(this->get_logger(), "Path planned successfully");
            // RCLCPP_INFO(this->get_logger(), "Generated Trajectory:\n%s", trajectory->toString().c_str());
        } else {
            response->result = false;
            RCLCPP_ERROR(this->get_logger(), "Path planning failed.");
        }
    }

    void clickedPointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received clicked point: (%.2f, %.2f) in frame '%s'",
                    msg->point.x, msg->point.y, msg->header.frame_id.c_str());

        if (!planner_) {
            RCLCPP_ERROR(this->get_logger(), "Planner is not initialized. Cannot validate point.");
            return;
        }

        // Create an OMPL state representing the clicked point
        auto space_info = planner_->getSpaceInformation();
        if (!space_info) {
            RCLCPP_ERROR(this->get_logger(), "SpaceInformation is not available. Cannot validate point.");
            return;
        }

        ompl::base::ScopedState<ompl::base::SE2StateSpace> ompl_state(space_info->getStateSpace());
        ompl_state->setX(msg->point.x);
        ompl_state->setY(msg->point.y);
        ompl_state->setYaw(0.0); // Default orientation

        // Check validity
        bool is_valid = space_info->isValid(ompl_state.get());

        // Log the result
        if (is_valid) {
            RCLCPP_INFO(this->get_logger(), "Clicked point (%.2f, %.2f) is VALID.",
                        ompl_state->getX(), ompl_state->getY());
        } else {
            RCLCPP_WARN(this->get_logger(), "Clicked point (%.2f, %.2f) is INVALID (Collision or out of bounds).",
                        ompl_state->getX(), ompl_state->getY());
        }
    }
    bool planning = false;
    double robot_radius_; // Robot radius as class member
    double turning_radius_; // Turning radius as class member
    std::shared_ptr<puzzlebot_planning::planners::OMPLPlanner> planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Service<puzzlebot_interfaces::srv::PlanPath>::SharedPtr service_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OMPLPlannerServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}