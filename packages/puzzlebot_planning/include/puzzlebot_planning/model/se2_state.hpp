#ifndef PUZZLEBOT_PLANNING__MODEL__SE2_STATE_HPP_
#define PUZZLEBOT_PLANNING__MODEL__SE2_STATE_HPP_

#include "puzzlebot_planning/model/state.hpp"
#include <string>
#include <cmath> // For std::sqrt, std::pow
#include <sstream> // For std::ostringstream
#include <iomanip> // For std::fixed, std::setprecision

namespace puzzlebot_planning
{
namespace model
{

  class SE2State;

using SE2StatePtr = std::shared_ptr<SE2State>;


/**
 * @brief Represents a state in SE(2) (x, y, theta).
 */
class SE2State : public State
{
public:
  /**
   * @brief Construct a new SE2State object.
   * @param x The x-coordinate.
   * @param y The y-coordinate.
   * @param theta The orientation angle in radians.
   */
  SE2State(double x, double y, double theta);

  // Default constructor (optional, but can be useful)
  SE2State();

  // Override the pure virtual functions from the base class
  std::string toString() const override;
  double distanceTo(const State& other) const override;
  StatePtr interpolate(const State& other, double t) const override;

  // --- Getters ---
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getTheta() const { return theta_; }

  // --- Setters ---
  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setTheta(double theta) { theta_ = theta; }

private:
  double x_;
  double y_;
  double theta_; // Angle in radians
};

} // namespace model
} // namespace puzzlebot_planning

#endif // PUZZLEBOT_PLANNING__MODEL__SE2_STATE_HPP_