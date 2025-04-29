#include "puzzlebot_planning/model/trajectory.hpp"
#include <sstream>

namespace puzzlebot_planning::model {

Trajectory::Trajectory() {}

void Trajectory::addState(const StatePtr& state) {
    states_.push_back(state);
}

const std::vector<StatePtr>& Trajectory::getStates() const {
    return states_;
}

size_t Trajectory::size() const {
    return states_.size();
}

void Trajectory::clear() {
    states_.clear();
}

std::string Trajectory::toString() const {
    std::ostringstream oss;
    oss << "Trajectory with " << states_.size() << " states:\n";
    for (size_t i = 0; i < states_.size(); ++i) {
        oss << "  [" << i << "] " << states_[i]->toString() << "\n";
    }
    return oss.str();
}

} // namespace puzzlebot_planning::model
