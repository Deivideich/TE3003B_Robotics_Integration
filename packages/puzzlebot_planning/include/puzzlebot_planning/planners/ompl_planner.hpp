#ifndef PUZZLEBOT_PLANNING_PLANNERS_OMPL_PLANNER_HPP_
#define PUZZLEBOT_PLANNING_PLANNERS_OMPL_PLANNER_HPP_

#include "puzzlebot_planning/planners/planner.hpp"
#include "puzzlebot_planning/state_validity/circle_map_validator.hpp"
#include "puzzlebot_planning/model/se2_state.hpp"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

namespace puzzlebot_planning::planners {

class OMPLPlanner : public Planner {
public:
    OMPLPlanner(double robot_radius, int occupancy_threshold);

    bool plan() override;

    TrajectoryPtr getTrajectory() const override { return trajectory_; }
    StatePtr getStart() const override { return start_; }
    StatePtr getGoal() const override { return goal_; }

    void setStart(const StatePtr& start);
    void setGoal(const StatePtr& goal);

    void updateMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map);

    void setPlanner(const std::string& planner_name);

    ompl::base::SpaceInformationPtr getSpaceInformation() const {
        return simple_setup_->getSpaceInformation();
    }

private:
    std::shared_ptr<ompl::geometric::SimpleSetup> simple_setup_;
    std::shared_ptr<ompl::base::DubinsStateSpace> state_space_;
    std::shared_ptr<ompl::base::SpaceInformation> space_info_;
    std::shared_ptr<ompl::base::Planner> planner_;

    std::shared_ptr<puzzlebot_planning::state_validity::CircleMapValidator> validator_;
    TrajectoryPtr trajectory_;
    SE2StatePtr start_;
    SE2StatePtr goal_;

};

} // namespace puzzlebot_planning::planners

#endif // PUZZLEBOT_PLANNING_PLANNERS_OMPL_PLANNER_HPP_