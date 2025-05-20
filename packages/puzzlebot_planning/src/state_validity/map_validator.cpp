#include "puzzlebot_planning/state_validity/map_validator.hpp"
#include <rclcpp/logging.hpp> // For logging warnings/errors

namespace puzzlebot_planning::state_validity {

// Helper function to get a logger (replace with your actual node's logger if available)
// This is a placeholder; ideally, pass a logger instance or use a static logger.
static rclcpp::Logger GetLogger() {
    return rclcpp::get_logger("map_validator");
}

// --- MapValidator Implementation ---

MapValidator::MapValidator(const ob::SpaceInformationPtr& si, int occupancy_threshold)
    : ob::StateValidityChecker(si),
      occupancy_threshold_(occupancy_threshold) {
    // Removed robot_radius check from base class
    if (occupancy_threshold_ < 0 || occupancy_threshold_ > 100) {
         RCLCPP_WARN(GetLogger(), "Occupancy threshold (%d) is outside the valid range [0, 100]. Clamping to [0, 100].", occupancy_threshold_);
         occupancy_threshold_ = std::max(0, std::min(100, occupancy_threshold_));
    }
}

void MapValidator::updateMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map) {
    if (!map) {
        RCLCPP_WARN(GetLogger(), "Received null map pointer in updateMap.");
        return;
    }
    std::lock_guard<std::mutex> lock(map_mutex_);
    current_map_ = map;
    RCLCPP_DEBUG(GetLogger(), "Map updated successfully.");
}

bool MapValidator::isValid(const ob::State* state) const {
    std::lock_guard<std::mutex> lock(map_mutex_); // Lock mutex here

    // if (!current_map_) {
    //     RCLCPP_WARN_THROTTLE(GetLogger(), *si_->getStateSpace()->getClock(), 5000, "Map not available for validity checking.");
    //     return false; // Cannot validate without a map
    // }

    // Cast the OMPL state to the expected SE2 state type
    const auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
    if (!se2_state) {
        RCLCPP_ERROR(GetLogger(), "State is not of type SE2StateSpace::StateType.");
        return false; // Should not happen if using the correct state space
    }

    // Call the (pure) virtual collision checking function implemented by the subclass
    return isCollisionFree(se2_state->getX(), se2_state->getY());
}

// Removed MapValidator::isCollisionFree implementation as it's now pure virtual

// Removed CircleMapValidator implementation from this file

} // namespace puzzlebot_planning::state_validity
