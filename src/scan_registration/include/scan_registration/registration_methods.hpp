#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

namespace scan_registration
{

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

struct RegistrationOptions
{
  std::string method = "VGICP";
  double resolution = 0.5;
  double transformation_epsilon = 1.0e-6;
  // Per-iteration rotation step below which fast_gicp declares convergence
  // (rotation-matrix element delta, ~radians for small angles). Convergence
  // requires the rotation AND translation criteria in the same iteration.
  // Only honored by FAST_GICP/VGICP; default equals fast_gicp's hardcoded
  // value so existing callers keep their behavior.
  double rotation_epsilon = 2.0e-3;
  int max_iterations = 30;
  double max_correspondence_distance = 0.5;
  int num_threads = 0;
  int correspondence_randomness = 20;
};

pcl::Registration<PointType, PointType>::Ptr create_registration(
  const RegistrationOptions & options);

// RMS distance in meters over source points whose nearest target neighbor is
// within max_range meters. Returns std::numeric_limits<double>::max() when no
// correspondence qualifies.
double calc_fitness_score(
  const CloudType::ConstPtr & target,
  const CloudType::ConstPtr & source,
  const Eigen::Isometry3d & target_from_source,
  double max_range);

}  // namespace scan_registration
