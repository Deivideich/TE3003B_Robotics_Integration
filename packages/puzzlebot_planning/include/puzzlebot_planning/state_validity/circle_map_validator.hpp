#ifndef PUZZLEBOT_PLANNING__STATE_VALIDITY__CIRCLE_MAP_VALIDATOR_HPP_
#define PUZZLEBOT_PLANNING__STATE_VALIDITY__CIRCLE_MAP_VALIDATOR_HPP_

#include "puzzlebot_planning/state_validity/map_validator.hpp" // Include the base class header

namespace puzzlebot_planning::state_validity {

/**
 * @brief OMPL State Validity Checker using an Occupancy Grid and a circular footprint.
 *
 * Extends MapValidator to implement collision checking for a circular robot.
 * Pre-calculates radius in cells when the map is updated.
 */
class CircleMapValidator : public MapValidator {
public:
    /**
     * @brief Constructor.
     * @param si The OMPL space information pointer.
     * @param robot_radius The radius of the robot's circular footprint (in meters).
     * @param occupancy_threshold The threshold (0-100) above which a cell is considered occupied.
     */
    CircleMapValidator(const ob::SpaceInformationPtr& si, double robot_radius, int occupancy_threshold = 50);

    /**
     * @brief Update the occupancy grid map and pre-calculate radius in cells.
     * This method is thread-safe.
     * @param map The new map.
     */
    void updateMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map) override;
    double getRobotRadius() const { return robot_radius_; }
protected:
    /**
     * @brief Check collision for a circular robot footprint at a given pose.
     * Uses pre-calculated radius in cells.
     * Assumes map_mutex_ is already locked by the caller (isValid in base class).
     * @param x Robot's x-coordinate (map frame).
     * @param y Robot's y-coordinate (map frame).
     * @return True if collision-free, false otherwise.
     */
    bool isCollisionFree(double x, double y) const override;

private:
    double robot_radius_;
    int radius_in_cells_ = 0; // Pre-calculated radius in grid cells
};

} // namespace puzzlebot_planning::state_validity

#endif // PUZZLEBOT_PLANNING__STATE_VALIDITY__CIRCLE_MAP_VALIDATOR_HPP_
