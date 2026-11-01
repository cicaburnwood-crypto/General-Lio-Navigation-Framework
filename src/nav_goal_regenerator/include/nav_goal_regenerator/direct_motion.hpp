#ifndef NAV_GOAL_REGENERATOR__DIRECT_MOTION_HPP_
#define NAV_GOAL_REGENERATOR__DIRECT_MOTION_HPP_

namespace nav_goal_regenerator
{

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Project the displacement from origin to current onto origin's fixed heading.
double projected_progress(
  const PlanarPose & origin,
  const PlanarPose & current);

// Proportional heading correction with wrapped angular error and symmetric limiting.
double heading_command(
  double target_heading,
  double current_heading,
  double proportional_gain,
  double max_angular_speed);

}  // namespace nav_goal_regenerator

#endif  // NAV_GOAL_REGENERATOR__DIRECT_MOTION_HPP_
