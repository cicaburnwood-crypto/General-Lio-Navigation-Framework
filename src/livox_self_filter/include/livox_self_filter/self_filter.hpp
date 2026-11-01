#ifndef LIVOX_SELF_FILTER__SELF_FILTER_HPP_
#define LIVOX_SELF_FILTER__SELF_FILTER_HPP_

#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "tf2/LinearMath/Transform.h"

namespace livox_self_filter
{

struct Box
{
  double min_x{0.0};
  double min_y{0.0};
  double min_z{0.0};
  double max_x{0.0};
  double max_y{0.0};
  double max_z{0.0};
};

tf2::Transform make_transform(
  const std::vector<double> & xyz,
  const std::vector<double> & rpy);

std::vector<Box> boxes_from_flat(const std::vector<double> & values);

bool point_inside_boxes(
  const livox_ros_driver2::msg::CustomPoint & point,
  const tf2::Transform & base_from_lidar,
  const std::vector<Box> & boxes);

livox_ros_driver2::msg::CustomMsg filter_message(
  const livox_ros_driver2::msg::CustomMsg & input,
  const tf2::Transform & base_from_lidar,
  const std::vector<Box> & boxes);

}  // namespace livox_self_filter

#endif  // LIVOX_SELF_FILTER__SELF_FILTER_HPP_
