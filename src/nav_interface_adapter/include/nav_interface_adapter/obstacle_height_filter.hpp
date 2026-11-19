#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"

namespace nav_interface_adapter
{

// Input coordinates are relative to the base origin. Only the rotation into
// height_frame is used: neither map translation nor absolute map height applies.
inline sensor_msgs::msg::PointCloud2 filter_obstacle_height(
  const sensor_msgs::msg::PointCloud2 & base_cloud,
  const tf2::Matrix3x3 & height_from_base_rotation,
  const double min_height, const double max_height)
{
  using Field = sensor_msgs::msg::PointField;
  if (!std::isfinite(min_height) || !std::isfinite(max_height) || min_height > max_height) {
    throw std::invalid_argument("obstacle height bounds must be finite and ordered");
  }
  const auto field = [&base_cloud](const char * name) -> const Field & {
      const auto it = std::find_if(
        base_cloud.fields.begin(), base_cloud.fields.end(),
        [name](const Field & item) {return item.name == name;});
      if (it == base_cloud.fields.end() || it->count != 1 ||
        (it->datatype != Field::FLOAT32 && it->datatype != Field::FLOAT64))
      {
        throw std::invalid_argument("obstacle cloud requires scalar floating-point XYZ fields");
      }
      const auto size = it->datatype == Field::FLOAT32 ? 4U : 8U;
      if (static_cast<std::size_t>(it->offset) + size > base_cloud.point_step) {
        throw std::invalid_argument("obstacle cloud has an invalid field offset");
      }
      return *it;
    };
  const auto & x_field = field("x");
  const auto & y_field = field("y");
  const auto & z_field = field("z");
  if (static_cast<std::size_t>(base_cloud.width) * base_cloud.point_step > base_cloud.row_step ||
    static_cast<std::size_t>(base_cloud.row_step) * base_cloud.height > base_cloud.data.size())
  {
    throw std::invalid_argument("obstacle cloud has an invalid data layout");
  }
  const uint16_t endian_probe = 1;
  const bool host_bigendian = *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
  const auto read = [&](const uint8_t * point, const Field & f) -> double {
      uint8_t bytes[8];
      const auto size = f.datatype == Field::FLOAT32 ? 4U : 8U;
      std::memcpy(bytes, point + f.offset, size);
      if (base_cloud.is_bigendian != host_bigendian) {
        std::reverse(bytes, bytes + size);
      }
      if (f.datatype == Field::FLOAT32) {
        float value;
        std::memcpy(&value, bytes, sizeof(value));
        return value;
      }
      double value;
      std::memcpy(&value, bytes, sizeof(value));
      return value;
    };

  auto output = base_cloud;
  output.height = 1;
  output.width = 0;
  output.row_step = 0;
  output.is_dense = true;
  output.data.clear();
  output.data.reserve(base_cloud.data.size());
  const auto up_in_base = height_from_base_rotation.getRow(2);
  for (uint32_t row = 0; row < base_cloud.height; ++row) {
    for (uint32_t col = 0; col < base_cloud.width; ++col) {
      const auto * point = base_cloud.data.data() +
        static_cast<std::size_t>(row) * base_cloud.row_step +
        static_cast<std::size_t>(col) * base_cloud.point_step;
      const double x = read(point, x_field);
      const double y = read(point, y_field);
      const double z = read(point, z_field);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const double height = up_in_base.dot(tf2::Vector3(x, y, z));
      if (std::isfinite(height) && height >= min_height && height <= max_height) {
        output.data.insert(output.data.end(), point, point + base_cloud.point_step);
        ++output.width;
      }
    }
  }
  output.row_step = output.width * output.point_step;
  return output;
}

}  // namespace nav_interface_adapter
