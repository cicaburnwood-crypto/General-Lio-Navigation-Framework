#include "scan_registration/registration_methods.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/kdtree.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

namespace scan_registration
{
namespace
{

std::string upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

}  // namespace

pcl::Registration<PointType, PointType>::Ptr create_registration(
  const RegistrationOptions & options)
{
  const auto method = upper(options.method);
  pcl::Registration<PointType, PointType>::Ptr registration;

  if (method == "ICP") {
    registration.reset(new pcl::IterativeClosestPoint<PointType, PointType>());
  } else if (method == "GICP") {
    registration.reset(new pcl::GeneralizedIterativeClosestPoint<PointType, PointType>());
  } else if (method == "NDT") {
    auto ndt = pcl::NormalDistributionsTransform<PointType, PointType>::Ptr(
      new pcl::NormalDistributionsTransform<PointType, PointType>());
    ndt->setResolution(options.resolution);
    registration = ndt;
  } else if (method == "GICP_OMP") {
    registration.reset(new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());
  } else if (method == "NDT_OMP") {
    auto ndt = pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr(
      new pclomp::NormalDistributionsTransform<PointType, PointType>());
    ndt->setResolution(options.resolution);
    registration = ndt;
  } else if (method == "FAST_GICP") {
    auto gicp = fast_gicp::FastGICP<PointType, PointType>::Ptr(
      new fast_gicp::FastGICP<PointType, PointType>());
    gicp->setNumThreads(options.num_threads);
    gicp->setCorrespondenceRandomness(options.correspondence_randomness);
    gicp->setRotationEpsilon(options.rotation_epsilon);
    registration = gicp;
  } else if (method == "VGICP") {
    auto vgicp = fast_gicp::FastVGICP<PointType, PointType>::Ptr(
      new fast_gicp::FastVGICP<PointType, PointType>());
    vgicp->setNumThreads(options.num_threads);
    vgicp->setResolution(options.resolution);
    vgicp->setCorrespondenceRandomness(options.correspondence_randomness);
    vgicp->setRotationEpsilon(options.rotation_epsilon);
    registration = vgicp;
  } else {
    throw std::invalid_argument("unknown registration method: " + options.method);
  }

  registration->setTransformationEpsilon(options.transformation_epsilon);
  registration->setMaximumIterations(options.max_iterations);
  registration->setMaxCorrespondenceDistance(options.max_correspondence_distance);
  return registration;
}

double calc_fitness_score(
  const CloudType::ConstPtr & target,
  const CloudType::ConstPtr & source,
  const Eigen::Isometry3d & target_from_source,
  double max_range)
{
  if (!target || !source || target->empty() || source->empty()) {
    return std::numeric_limits<double>::max();
  }

  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
  tree->setInputCloud(target);

  CloudType source_transformed;
  pcl::transformPointCloud(*source, source_transformed, target_from_source.cast<float>());

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);
  const double max_range_sq = max_range * max_range;
  double squared_error_sum = 0.0;
  int accepted = 0;

  for (const auto & point : source_transformed.points) {
    if (tree->nearestKSearch(point, 1, nn_indices, nn_dists) <= 0) {
      continue;
    }
    // KdTree returns squared distances.
    if (nn_dists[0] <= max_range_sq) {
      squared_error_sum += nn_dists[0];
      ++accepted;
    }
  }

  return accepted > 0 ?
    std::sqrt(squared_error_sum / accepted) :
    std::numeric_limits<double>::max();
}

}  // namespace scan_registration
