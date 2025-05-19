#include "puzzlebot_planning/planners/ompl_planner.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <rclcpp/rclcpp.hpp>


namespace puzzlebot_planning::planners {

OMPLPlanner::OMPLPlanner(double robot_radius, int occupancy_threshold) {
    RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "Initializing OMPLPlanner...");

    auto space = std::make_shared<ompl::base::ReedsSheppStateSpace>();
    if (!space) {
        RCLCPP_ERROR(rclcpp::get_logger("OMPLPlanner"), "Failed to create SE2StateSpace.");
        throw std::runtime_error("Failed to create SE2StateSpace");
    }
    RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "SE2StateSpace created successfully.");

    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-50); // Example bounds
    bounds.setHigh(50);
    space->setBounds(bounds);
    RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "Bounds set to: [%.2f, %.2f]", bounds.low[0], bounds.high[0]);

    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(space);
    if (!simple_setup_) {
        RCLCPP_ERROR(rclcpp::get_logger("OMPLPlanner"), "Failed to create SimpleSetup.");
        throw std::runtime_error("Failed to create SimpleSetup");
    }
    RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "SimpleSetup created successfully.");

    validator_ = std::make_shared<puzzlebot_planning::state_validity::CircleMapValidator>(
        simple_setup_->getSpaceInformation(), robot_radius, occupancy_threshold);
    if (!validator_) {
        RCLCPP_ERROR(rclcpp::get_logger("OMPLPlanner"), "Failed to create CircleMapValidator.");
        throw std::runtime_error("Failed to create CircleMapValidator");
    }
    RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "CircleMapValidator created successfully with robot_radius: %.2f and occupancy_threshold: %d", robot_radius, occupancy_threshold);

    // Ensure the validator is set up correctly
    simple_setup_->setStateValidityChecker([validator = validator_](const ompl::base::State* state) {
        if (!validator) {
            RCLCPP_ERROR(rclcpp::get_logger("OMPLPlanner"), "Validator is null in state validity checker.");
            return false;
        }
        return validator->isValid(state);
    });
    RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "State validity checker set successfully.");

    // simple_setup_->setup();
    // RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "SimpleSetup setup completed.");

    trajectory_ = std::make_shared<Trajectory>();
    if (!trajectory_) {
        RCLCPP_ERROR(rclcpp::get_logger("OMPLPlanner"), "Failed to create Trajectory.");
        throw std::runtime_error("Failed to create Trajectory");
    }
    RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "Trajectory object created successfully.");

    // Log initialization success
    RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "OMPLPlanner initialized successfully.");
}

void OMPLPlanner::updateMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map) {
    validator_->updateMap(map);
}

void OMPLPlanner::setStart(const StatePtr& start) {
    start_ = std::dynamic_pointer_cast<SE2State>(start);
}

void OMPLPlanner::setGoal(const StatePtr& goal) {
    goal_ = std::dynamic_pointer_cast<SE2State>(goal);
}

void OMPLPlanner::setPlanner(const std::string& planner_name) {
    ompl::base::PlannerPtr planner;

    if (planner_name == "RRT") {
        planner = std::make_shared<ompl::geometric::RRT>(simple_setup_->getSpaceInformation());
    } else if (planner_name == "RRTConnect") {
        planner = std::make_shared<ompl::geometric::RRTConnect>(simple_setup_->getSpaceInformation());
    } else if (planner_name == "PRM") {
        planner = std::make_shared<ompl::geometric::PRM>(simple_setup_->getSpaceInformation());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("OMPLPlanner"), "Unknown planner name: %s", planner_name.c_str());
        return;
    }

    simple_setup_->setPlanner(planner);
    RCLCPP_INFO(rclcpp::get_logger("OMPLPlanner"), "Planner set to: %s", planner_name.c_str());
}

bool OMPLPlanner::plan() {
    if (!start_ || !goal_) {
        RCLCPP_ERROR(rclcpp::get_logger("OMPLPlanner"), "Start or goal state is not set.");
        return false;
    }

    auto space = simple_setup_->getStateSpace()->as<ompl::base::ReedsSheppStateSpace>();
    ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> ompl_start(simple_setup_->getSpaceInformation());
    ompl_start->setX(start_->getX());
    ompl_start->setY(start_->getY());
    ompl_start->setYaw(start_->getTheta());

    ompl::base::ScopedState<ompl::base::ReedsSheppStateSpace> ompl_goal(simple_setup_->getSpaceInformation());
    ompl_goal->setX(goal_->getX());
    ompl_goal->setY(goal_->getY());
    ompl_goal->setYaw(goal_->getTheta());

    simple_setup_->setStartAndGoalStates(ompl_start, ompl_goal);

    if (simple_setup_->solve(1.0)) {
        auto path = simple_setup_->getSolutionPath();
        path.interpolate(100); // Interpolate to have 100 states in the trajectory
        trajectory_->clear();
        for (size_t i = 0; i < path.getStateCount(); ++i) {
            auto state = path.getState(i)->as<ompl::base::ReedsSheppStateSpace::StateType>();
            auto se2_state = std::make_shared<SE2State>(state->getX(), state->getY(), state->getYaw());
            trajectory_->addState(se2_state);
        }
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("OMPLPlanner"), "Failed to find a solution.");
        return false;
    }
}

} // namespace puzzlebot_planning::planners