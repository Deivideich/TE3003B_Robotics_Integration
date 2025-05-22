#ifndef PUZZLEBOT_PLANNING__MODEL__STATE_HPP_
#define PUZZLEBOT_PLANNING__MODEL__STATE_HPP_

#include <string>
#include <vector>
#include <memory> // For std::shared_ptr

namespace puzzlebot_planning
{
namespace model
{

// Forward declare the class
class State;

// Define type aliases using the forward-declared class
using StatePtr = std::shared_ptr<State>;
using StateConstPtr = std::shared_ptr<const State>;

/**
 * @brief Abstract base class for representing a state in the planning space.
 */
class State
{
public:
  // Virtual destructor is important for base classes with virtual functions
  virtual ~State() = default;

  /**
   * @brief Returns a string representation of the state.
   * @return std::string String representation.
   */
  virtual std::string toString() const = 0; // Pure virtual function

  /**
   * @brief Calculate the distance to another state.
   * @param other The other state.
   * @return double The calculated distance.
   */
  virtual double distanceTo(const State& other) const = 0;

  /**
   * @brief Interpolate between this state and another state.
   * @param other The other state.
   * @param t Interpolation factor (0.0 = this state, 1.0 = other state).
   * @return StatePtr A pointer to the new interpolated state.
   * @note The caller is responsible for managing the memory of the returned pointer if raw pointers are used. Using StatePtr (shared_ptr) is recommended.
   */
  virtual StatePtr interpolate(const State& other, double t) const = 0;
};

} // namespace model
} // namespace puzzlebot_planning

#endif // PUZZLEBOT_PLANNING__MODEL__STATE_HPP_