#ifndef PUZZLEBOT_PLANNING__MODEL__TRAJECTORY_HPP_
#define PUZZLEBOT_PLANNING__MODEL__TRAJECTORY_HPP_

#include <vector>
#include <string>
#include "puzzlebot_planning/model/state.hpp" // Includes StatePtr




namespace puzzlebot_planning::model {
    class Trajectory;

    using TrajectoryPtr = std::shared_ptr<Trajectory>;
    using TrajectoryConstPtr = std::shared_ptr<const Trajectory>;

    /**
     * @brief Represents a sequence of states forming a trajectory.
     */
    class Trajectory {
    public:
        /**
         * @brief Default constructor. Creates an empty trajectory.
         */
        Trajectory();

        /**
         * @brief Adds a state to the end of the trajectory.
         * @param state Shared pointer to the state to add.
         */
        void addState(const StatePtr& state);

        /**
         * @brief Gets the sequence of states in the trajectory.
         * @return Const reference to the vector of StatePtr.
         */
        const std::vector<StatePtr>& getStates() const;

        /**
         * @brief Gets the number of states in the trajectory.
         * @return The size of the trajectory.
         */
        size_t size() const;

        /**
         * @brief Clears all states from the trajectory.
         */
        void clear();

        /**
         * @brief Generates a string representation of the trajectory.
         * @return String representation.
         */
        std::string toString() const;

    private:
        std::vector<StatePtr> states_;
    };

} // namespace puzzlebot_planning::model

#endif // PUZZLEBOT_PLANNING__MODEL__TRAJECTORY_HPP_
