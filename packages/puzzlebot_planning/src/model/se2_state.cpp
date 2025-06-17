#include "puzzlebot_planning/model/se2_state.hpp"
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <memory> // For std::make_shared

namespace puzzlebot_planning::model {

SE2State::SE2State(double x, double y, double theta)
    : x_(x), y_(y), theta_(theta) {}

SE2State::SE2State()
    : x_(0.0), y_(0.0), theta_(0.0) {}

double SE2State::distanceTo(const State& other) const {
    const auto* other_se2 = dynamic_cast<const SE2State*>(&other);
    if (!other_se2) {
        throw std::runtime_error("Cannot calculate distance between SE2State and incompatible State type.");
    }

    return std::sqrt(std::pow(x_ - other_se2->x_, 2) + std::pow(y_ - other_se2->y_, 2));
}

StatePtr SE2State::interpolate(const State& other, double t) const {
    const auto* other_se2 = dynamic_cast<const SE2State*>(&other);
    if (!other_se2) {
        throw std::runtime_error("Cannot interpolate between SE2State and incompatible State type.");
    }

    if (t < 0.0 || t > 1.0) {
        throw std::out_of_range("Interpolation factor t must be between 0.0 and 1.0");
    }

    double new_x = x_ + (other_se2->x_ - x_) * t;
    double new_y = y_ + (other_se2->y_ - y_) * t;
    double new_theta = theta_ + (other_se2->theta_ - theta_) * t;

    return std::make_shared<SE2State>(new_x, new_y, new_theta);
}

std::string SE2State::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3); // Format output
    oss << "SE2State(x=" << x_ << ", y=" << y_ << ", theta=" << theta_ << ")";
    return oss.str();
}

} // namespace puzzlebot_planning::model