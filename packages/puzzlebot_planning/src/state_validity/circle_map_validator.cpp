#include "puzzlebot_planning/state_validity/circle_map_validator.hpp"
#include <rclcpp/logging.hpp> // For logging warnings/errors
#include <algorithm> // For std::max, std::min

namespace puzzlebot_planning::state_validity {

// Helper function to get a logger (replace with your actual node's logger if available)
static rclcpp::Logger GetCircleLogger() {
    return rclcpp::get_logger("circle_map_validator");
}

CircleMapValidator::CircleMapValidator(const ob::SpaceInformationPtr& si, double robot_radius, int occupancy_threshold)
    : MapValidator(si, occupancy_threshold),
      robot_radius_(robot_radius) {
    if (robot_radius_ <= 0) {
        RCLCPP_WARN(GetCircleLogger(), "Robot radius is non-positive (%.3f). Collision checking might be ineffective.", robot_radius_);
    }
    // Initialize radius_in_cells_ based on an initial (potentially null) map
    // It will be properly calculated when updateMap is called with a valid map.
    radius_in_cells_ = 0;
}

void CircleMapValidator::updateMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map) {
    // Call base class method first to update the map pointer under lock
    MapValidator::updateMap(map);

    // Now, calculate radius_in_cells_ using the potentially updated map
    // Lock again to safely access current_map_ and update radius_in_cells_
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (current_map_ && current_map_->info.resolution > 0) {
        const double resolution = current_map_->info.resolution;
        const double radius_in_cells_d = (robot_radius_ / resolution) + 1e-3; // Add epsilon
        radius_in_cells_ = static_cast<int>(std::ceil(radius_in_cells_d));
        RCLCPP_DEBUG(GetCircleLogger(), "Map updated. Resolution: %.3f, Robot Radius: %.3f, Radius in Cells: %d",
                     resolution, robot_radius_, radius_in_cells_);
    } else {
        // Handle cases where map is null or resolution is invalid after update
        radius_in_cells_ = 0; // Set to 0 or some indicator of invalidity
        if (current_map_ && current_map_->info.resolution <= 0) {
             RCLCPP_ERROR(GetCircleLogger(), "Map resolution is non-positive (%.3f) after update. Cannot calculate radius in cells.", current_map_->info.resolution);
        }
         // Base class updateMap already warns if map is null
    }
}

bool CircleMapValidator::isCollisionFree(double x, double y) const {
    // Assumes map_mutex_ is locked by the caller (isValid in base class)
    // Base class isValid checks if current_map_ is valid

    // Use the pre-calculated radius_in_cells_
    // If radius_in_cells_ is 0 (e.g., due to invalid map resolution), treat as collision
    // if (radius_in_cells_ <= 0) {
    //      RCLCPP_WARN_THROTTLE(GetCircleLogger(), *si_->getStateSpace()->getClock(), 5000, "Radius in cells is not valid (%d). Assuming collision.", radius_in_cells_);
    //      return false;
    // }

    // std::cout << "CircleMapValidator::isCollisionFree called with x: " << x << ", y: " << y << std::endl;

    const auto& map_info = current_map_->info;
    const double resolution = map_info.resolution; // Already checked > 0 in updateMap
    const double origin_x = map_info.origin.position.x;
    const double origin_y = map_info.origin.position.y;
    const unsigned int width = map_info.width;
    const unsigned int height = map_info.height;

    // Convert world coordinates (x, y) to map pixel coordinates (mx, my)
    const int center_mx = static_cast<int>((x - origin_x) / resolution);
    const int center_my = static_cast<int>((y - origin_y) / resolution);

    // Iterate using the pre-calculated radius_in_cells_
    for (int dy = -radius_in_cells_; dy <= radius_in_cells_; ++dy) {
        for (int dx = -radius_in_cells_; dx <= radius_in_cells_; ++dx) {
            // Check if the cell center is within the circular radius
            if (dx * dx + dy * dy > radius_in_cells_ * radius_in_cells_) {
                continue; // Skip cells outside the circle
            }

            const int current_mx = center_mx + dx;
            const int current_my = center_my + dy;

            // Check map boundaries
            if (current_mx < 0 || current_mx >= static_cast<int>(width) ||
                current_my < 0 || current_my >= static_cast<int>(height)) {
                return false; // Collision (outside map)
            }

            // Calculate the 1D index into the map data array
            const unsigned int map_index = current_my * width + current_mx;

            // Check the occupancy value
            const int8_t occupancy_value = current_map_->data[map_index];

            // -1 means unknown, treat as occupied for safety
            if (occupancy_value == -1 || occupancy_value >= occupancy_threshold_) {
                return false; // Collision detected
            }
        }
    }

    return true; // Collision-free
}

} // namespace puzzlebot_planning::state_validity
