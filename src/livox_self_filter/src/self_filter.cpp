#include "livox_self_filter/self_filter.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace livox_self_filter
{

tf2::Transform make_transform(
  const std::vector<double> & xyz,
  const std::vector<double> & rpy)
{
  if (xyz.size() != 3 || rpy.size() != 3) {
    throw std::invalid_argument("base_to_lidar_xyz and base_to_lidar_rpy must have 3 values");
  }

  tf2::Quaternion rotation;
  rotation.setRPY(rpy[0], rpy[1], rpy[2]);
  rotation.normalize();
  return tf2::Transform(rotation, tf2::Vector3(xyz[0], xyz[1], xyz[2]));
}

std::vector<Box> boxes_from_flat(const std::vector<double> & values)
{
  if (values.size() % 6 != 0) {
    throw std::invalid_argument("boxes_base_link must contain 6 values per box");
  }

  std::vector<Box> boxes;
  boxes.reserve(values.size() / 6);
  for (std::size_t i = 0; i < values.size(); i += 6) {
    Box box;
    box.min_x = std::min(values[i + 0], values[i + 3]);
    box.min_y = std::min(values[i + 1], values[i + 4]);
    box.min_z = std::min(values[i + 2], values[i + 5]);
    box.max_x = std::max(values[i + 0], values[i + 3]);
    box.max_y = std::max(values[i + 1], values[i + 4]);
    box.max_z = std::max(values[i + 2], values[i + 5]);
    boxes.push_back(box);
  }
  return boxes;
}

bool point_inside_boxes(
  const livox_ros_driver2::msg::CustomPoint & point,
  const tf2::Transform & base_from_lidar,
  const std::vector<Box> & boxes)
{
  if (boxes.empty()) {
    return false;
  }

  const auto point_base =
    base_from_lidar * tf2::Vector3(point.x, point.y, point.z);
  const double x = point_base.x();
  const double y = point_base.y();
  const double z = point_base.z();

  for (const auto & box : boxes) {
    if (x >= box.min_x && x <= box.max_x &&
      y >= box.min_y && y <= box.max_y &&
      z >= box.min_z && z <= box.max_z)
    {
      return true;
    }
  }
  return false;
}

livox_ros_driver2::msg::CustomMsg filter_message(
  const livox_ros_driver2::msg::CustomMsg & input,
  const tf2::Transform & base_from_lidar,
  const std::vector<Box> & boxes)
{
  livox_ros_driver2::msg::CustomMsg output;
  output.header = input.header;
  output.timebase = input.timebase;
  output.lidar_id = input.lidar_id;
  output.rsvd = input.rsvd;
  output.points.reserve(input.points.size());

  for (const auto & point : input.points) {
    if (!point_inside_boxes(point, base_from_lidar, boxes)) {
      output.points.push_back(point);
    }
  }

  output.point_num = static_cast<std::uint32_t>(output.points.size());
  return output;
}

}  // namespace livox_self_filter
