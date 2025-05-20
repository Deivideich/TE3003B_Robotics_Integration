#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "puzzlebot_interfaces/srv/plan_path.hpp"
#include "puzzlebot_planning/planners/astar_planner.hpp"
#include "puzzlebot_planning/model/trajectory.hpp"
#include "puzzlebot_planning/model/se2_state.hpp"
#include "puzzlebot_planning/model/state.hpp"

#include <unordered_map>
#include <cmath>
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class AStarPlannerServer : public rclcpp::Node
{
    private:
        rclcpp::Service<puzzlebot_interfaces::srv::PlanPath>::SharedPtr plan_path_service_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

        // A* planner instance
        std::shared_ptr<puzzlebot_planning::planners::AStarPlanner> astar_planner_;

        // Paramaters
        float theta_resolution_;
        float translational_weight_;
        float rotational_weight_;
        int interpolation_steps_;
        bool using_real_sampling_;
        float origin_x, origin_y;
        float robot_width_, robot_height_;
    public:
        AStarPlannerServer() : Node("astar_planner_server") {
            // Declare and initialize parameters
            declare_parameters();
            initialize_parameters();
        }

        void setup(){
            // Initialize subscribers
            plan_path_service_ = this->create_service<puzzlebot_interfaces::srv::PlanPath>(
                "plan_path", std::bind(&AStarPlannerServer::planPathCallback, this, std::placeholders::_1, std::placeholders::_2));
            path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
            
            // Wait for occupancy grid message
            auto message = nav_msgs::msg::OccupancyGrid();
            RCLCPP_INFO(this->get_logger(), "Waiting for map message");
            while (!rclcpp::wait_for_message(message, this->shared_from_this(),"/map",std::chrono::seconds(1)));
            RCLCPP_INFO(this->get_logger(), "Map message received");
            
            RCLCPP_INFO(this->get_logger(), "A* Planner Server initialized with parameters: "
                "map_resolution=%0.2f theta_resolution=%.2f, translational_weight=%.2f, "
                "rotational_weight=%.2f, interpolation_steps=%d, using_real_sampling=%s",
                message.info.resolution, theta_resolution_, translational_weight_, rotational_weight_,
                interpolation_steps_, using_real_sampling_ ? "true" : "false");

            // Convert occupancy grid to 2D grid map
            Grid grid_map(message.info.height, std::vector<int>(message.info.width, 0));
            for (size_t i = 0; i < message.info.height; ++i) {
                for (size_t j = 0; j < message.info.width; ++j) {
                    int value = message.data[i * message.info.width + j];
                    if (message.data[i * message.info.width + j] == -1 || message.data[i * message.info.width + j] == 100) {
                        grid_map[i][j] = 1; // Unknown or occupied space
                    }
                }
            }

            origin_x = message.info.origin.position.x;
            origin_y = message.info.origin.position.y;

            std::vector<std::pair<float,float>> base_footprint = {
                {-robot_width_ / 2.0f, -robot_height_ / 2.0f},  // bottom-left
                { robot_width_ / 2.0f,  robot_height_ / 2.0f}   // top-right
            };
                        
            // Initialize A* planner
            astar_planner_ = std::make_shared<puzzlebot_planning::planners::AStarPlanner>(
                grid_map, base_footprint, message.info.resolution, message.info.origin.position.x, message.info.origin.position.y
                , theta_resolution_, translational_weight_, rotational_weight_, interpolation_steps_);

            astar_planner_->setUsingRealSampling(using_real_sampling_);
            RCLCPP_INFO(this->get_logger(), "A* Planner initialized with grid size: %zu x %zu", grid_map.size(), grid_map[0].size());
            RCLCPP_INFO(this->get_logger(), "A* Planner ready to plan paths");
        }

        void declare_parameters(){
            this->declare_parameter("theta_resolution", M_PI / 8);
            this->declare_parameter("translational_weight", 0.5f);
            this->declare_parameter("rotational_weight", 0.5f);
            this->declare_parameter("interpolation_steps", 100);
            this->declare_parameter("using_real_sampling", false);
            this->declare_parameter("robot_width", 0.4f);
            this->declare_parameter("robot_height", 0.4f);
        }

        void initialize_parameters(){
            this->get_parameter("theta_resolution", theta_resolution_);
            this->get_parameter("translational_weight", translational_weight_);
            this->get_parameter("rotational_weight", rotational_weight_);
            this->get_parameter("interpolation_steps", interpolation_steps_);
            this->get_parameter("using_real_sampling", using_real_sampling_);
            this->get_parameter("robot_width", robot_width_);
            this->get_parameter("robot_height", robot_height_);
        }

        void planPathCallback(const std::shared_ptr<puzzlebot_interfaces::srv::PlanPath::Request> request,
                            const std::shared_ptr<puzzlebot_interfaces::srv::PlanPath::Response> response){
            // Set start and goal states
            double yaw, pitch, roll;       

            // Convert quaternion to roll, pitch, yaw START POSE
            geometry_msgs::msg::Quaternion start_quat = request->start.pose.orientation;
            tf2::Quaternion tf_start(start_quat.x, start_quat.y, start_quat.z, start_quat.w); 
            tf2::Matrix3x3(tf_start).getRPY(roll, pitch, yaw);
            auto start_state = std::make_shared<puzzlebot_planning::model::SE2State>(request->start.pose.position.x, request->start.pose.position.y, yaw);

            geometry_msgs::msg::Quaternion goal_quat = request->goal.pose.orientation;
            tf2::Quaternion tf_goal(goal_quat.x, goal_quat.y, goal_quat.z, goal_quat.w); 
            tf2::Matrix3x3(tf_goal).getRPY(roll, pitch, yaw);
            auto goal_state = std::make_shared<puzzlebot_planning::model::SE2State>(request->goal.pose.position.x, request->goal.pose.position.y, yaw);

            astar_planner_->setStart(start_state);
            astar_planner_->setGoal(goal_state);
            

            int start_x = static_cast<int>((request->start.pose.position.x - origin_x) / astar_planner_->getMapResolution());
            int start_y = static_cast<int>((request->start.pose.position.y - origin_y) / astar_planner_->getMapResolution());
            int goal_x = static_cast<int>((request->goal.pose.position.x - origin_x) / astar_planner_->getMapResolution());
            int goal_y = static_cast<int>((request->goal.pose.position.y - origin_y) / astar_planner_->getMapResolution());

            // Check if start and goal are within the grid bounds
            if (start_x < 0 || start_x >= astar_planner_->getGrid()[0].size() ||
                start_y < 0 || start_y >= astar_planner_->getGrid().size() ||
                goal_x < 0 || goal_x >= astar_planner_->getGrid()[0].size() ||
                goal_y < 0 || goal_y >= astar_planner_->getGrid().size()) {
                RCLCPP_ERROR(this->get_logger(), "Start or goal position is out of grid bounds");
                response->result = false;
                return;
            }

            // Find the path
            bool result = astar_planner_->plan();

            if (result) {
                // Get the trajectory
                auto trajectory = astar_planner_->getTrajectory();
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
                RCLCPP_ERROR(this->get_logger(), "Failed to find a path");
            }
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlannerServer>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}