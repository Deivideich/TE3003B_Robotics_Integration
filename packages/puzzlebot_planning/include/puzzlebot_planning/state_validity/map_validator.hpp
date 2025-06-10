#ifndef PUZZLEBOT_PLANNING__STATE_VALIDITY__MAP_VALIDATOR_HPP_
#define PUZZLEBOT_PLANNING__STATE_VALIDITY__MAP_VALIDATOR_HPP_

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <mutex>
#include <cmath>
#include <memory> // For std::shared_ptr

namespace puzzlebot_planning::state_validity {

namespace ob = ompl::base;

/**
 * @brief Base OMPL State Validity Checker using an Occupancy Grid.
 *
 * Provides map update functionality and basic validity checks.
 * Collision checking logic is delegated to subclasses.
 */
class MapValidator : public ob::StateValidityChecker {
public:
    /**
     * @brief Constructor.
     * @param si The OMPL space information pointer.
     * @param occupancy_threshold The threshold (0-100) above which a cell is considered occupied.
     */
    MapValidator(const ob::SpaceInformationPtr& si, int occupancy_threshold = 50);

    /**
     * @brief Update the occupancy grid map used for collision checking.
     * This method is thread-safe.
     * @param map The new map.
     */
    virtual void updateMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map);

    /**
     * @brief Check if a given state is valid.
     * Checks map availability and state type, then calls the subclass's isCollisionFree.
     * This method is thread-safe.
     * @param state The state to check (assumed to be SE2).
     * @return True if the state is valid, false otherwise.
     */
    bool isValid(const ob::State* state) const override;

protected: // Changed to protected for subclass access
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr current_map_;
    mutable std::mutex map_mutex_; // Protects access to current_map_
    int occupancy_threshold_;

    /**
     * @brief Pure virtual function for collision checking logic.
     * Subclasses must implement this to define the robot's footprint and collision rules.
     * Assumes map_mutex_ is already locked by the caller (isValid).
     * @param x Robot's x-coordinate (map frame).
     * @param y Robot's y-coordinate (map frame).
     * @return True if collision-free, false otherwise.
     */
    virtual bool isCollisionFree(double x, double y) const = 0;
};

} // namespace puzzlebot_planning::state_validity

#endif // PUZZLEBOT_PLANNING__STATE_VALIDITY__MAP_VALIDATOR_HPP_
