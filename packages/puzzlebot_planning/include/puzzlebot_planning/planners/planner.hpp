#ifndef PUZZLEBOT_PLANNING_PLANNERS_PLANNER_HPP_
#define PUZZLEBOT_PLANNING_PLANNERS_PLANNER_HPP_

#include "puzzlebot_planning/model/trajectory.hpp"
#include "puzzlebot_planning/model/state.hpp"
#include <memory>
using namespace puzzlebot_planning::model;
namespace puzzlebot_planning::planners {

class Planner {
public:
    virtual ~Planner() = default;

    // Pure virtual method to plan a path
    virtual bool plan() = 0;

    // Getters
    virtual TrajectoryPtr getTrajectory() const = 0;
    virtual StatePtr getStart() const = 0;
    virtual StatePtr getGoal() const = 0;

    // Setters
    virtual void setStart(const StatePtr& start) = 0;
    virtual void setGoal(const StatePtr& goal) = 0;
};

} // namespace puzzlebot_planning::planners

#endif // PUZZLEBOT_PLANNING_PLANNERS_PLANNER_HPP_