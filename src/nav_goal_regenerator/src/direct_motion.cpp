#include "nav_goal_regenerator/direct_motion.hpp"

#include <algorithm>
#include <cmath>

namespace nav_goal_regenerator
{

double projected_progress(
  const PlanarPose & origin,
  const PlanarPose & current)
{
  const auto dx = current.x - origin.x;
  const auto dy = current.y - origin.y;
  return dx * std::cos(origin.yaw) + dy * std::sin(origin.yaw);
}

double heading_command(
  const double target_heading,
  const double current_heading,
  const double proportional_gain,
  const double max_angular_speed)
{
  const auto heading_error = std::atan2(
    std::sin(target_heading - current_heading),
    std::cos(target_heading - current_heading));
  const auto command_limit = std::abs(max_angular_speed);
  return std::clamp(
    proportional_gain * heading_error,
    -command_limit,
    command_limit);
}

}  // namespace nav_goal_regenerator
