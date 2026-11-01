#ifndef CHASSIS_3D_LOCALIZATION__CORRECTION_SMOOTHING_HPP_
#define CHASSIS_3D_LOCALIZATION__CORRECTION_SMOOTHING_HPP_

#include <algorithm>

#include <Eigen/Geometry>

namespace chassis_3d_localization
{

// Exponentially blends the tracked map->odom transform toward a newly
// accepted alignment result instead of overwriting it wholesale.
//
// Each scan-to-map solve carries scatter (weak yaw geometry in small rooms),
// and a wholesale overwrite injects that scatter 1:1 into the robot pose -
// visible as yaw twitches while standing still. Blending is an EMA: per-solve
// noise is attenuated by alpha, while a persistent offset is still absorbed
// geometrically (the remaining error shrinks by (1 - alpha) per accepted
// solve), so it can neither accumulate nor oscillate. Callers snap
// (alpha >= 1) on initialization and jump overrides, where gliding through
// intermediate poses would be wrong.
inline Eigen::Matrix4f blend_map_from_odom(
  const Eigen::Matrix4f & current,
  const Eigen::Matrix4f & target,
  const double alpha)
{
  if (alpha >= 1.0) {
    return target;
  }
  const float ratio = static_cast<float>(std::max(alpha, 0.0));

  Eigen::Quaternionf current_q(current.block<3, 3>(0, 0));
  Eigen::Quaternionf target_q(target.block<3, 3>(0, 0));
  current_q.normalize();
  target_q.normalize();

  Eigen::Matrix4f blended = Eigen::Matrix4f::Identity();
  blended.block<3, 3>(0, 0) =
    current_q.slerp(ratio, target_q).normalized().toRotationMatrix();
  blended.block<3, 1>(0, 3) =
    (1.0F - ratio) * current.block<3, 1>(0, 3) + ratio * target.block<3, 1>(0, 3);
  return blended;
}

}  // namespace chassis_3d_localization

#endif  // CHASSIS_3D_LOCALIZATION__CORRECTION_SMOOTHING_HPP_
