#include <array>
#include <limits>
#include <vector>
#include "gtest/gtest.h"
#include "nav_interface_adapter/obstacle_height_filter.hpp"

namespace
{
using Cloud = sensor_msgs::msg::PointCloud2;
using Point = std::array<double, 3>;
using nav_interface_adapter::filter_obstacle_height;

Cloud make_cloud(const std::vector<Point> & points)
{
  Cloud cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp.sec = 123;
  cloud.height = 1;
  cloud.width = points.size();
  cloud.point_step = 32;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  const uint16_t probe = 1;
  cloud.is_bigendian = *reinterpret_cast<const uint8_t *>(&probe) == 0;
  for (unsigned int i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = std::array<const char *, 4>{"x", "y", "z", "intensity"}[i];
    field.offset = i * 8;
    field.datatype = sensor_msgs::msg::PointField::FLOAT64;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(cloud.data.data() + i * 32, points[i].data(), 24);
    const double intensity = 100 + i;
    std::memcpy(cloud.data.data() + i * 32 + 24, &intensity, 8);
  }
  return cloud;
}

tf2::Matrix3x3 identity()
{
  tf2::Matrix3x3 rotation;
  rotation.setIdentity();
  return rotation;
}

TEST(ObstacleHeight, InclusiveBoundsAndNonfinitePoints)
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  auto cloud = make_cloud({{1, 0, -0.01}, {1, 0, 0}, {1, 0, 0.8},
      {1, 0, 0.81}, {nan, 0, 0.4}, {1, 0, nan}});
  const auto filtered = filter_obstacle_height(cloud, identity(), 0, 0.8);
  ASSERT_EQ(filtered.width, 2U);
  EXPECT_EQ(filtered.header, cloud.header);
  EXPECT_EQ(filtered.fields, cloud.fields);
  EXPECT_EQ(filtered.row_step, 64U);
  EXPECT_TRUE(filtered.is_dense);
  EXPECT_EQ(filtered.data, std::vector<uint8_t>(cloud.data.begin() + 32, cloud.data.begin() + 96));
}

TEST(ObstacleHeight, UsesMapAxisInsteadOfBodyZ)
{
  // map-up is base-X: body-Z is deliberately outside the permitted band.
  const tf2::Matrix3x3 rotation(0, 0, -1, 0, 1, 0, 1, 0, 0);
  const auto cloud = make_cloud({{0.4, 0, 9}, {0.9, 0, 0.4}, {-0.1, 0, 0.4}});
  const auto filtered = filter_obstacle_height(cloud, rotation, 0, 0.8);
  ASSERT_EQ(filtered.width, 1U);
  EXPECT_EQ(filtered.data, std::vector<uint8_t>(cloud.data.begin(), cloud.data.begin() + 32));
}

TEST(ObstacleHeight, ConfigurableRange)
{
  const auto cloud = make_cloud({{0, 0, 0.2}, {0, 0, 0.5}, {0, 0, 0.9}});
  EXPECT_EQ(filter_obstacle_height(cloud, identity(), 0.4, 1.0).width, 2U);
  EXPECT_EQ(filter_obstacle_height(cloud, identity(), 0.5, 0.5).width, 1U);
}

TEST(ObstacleHeight, OrganizedCloudWithRowPadding)
{
  auto cloud = make_cloud({{0, 0, 0.2}, {0, 0, 1.0}});
  const auto second = std::vector<uint8_t>(cloud.data.begin() + 32, cloud.data.end());
  cloud.width = 1;
  cloud.height = 2;
  cloud.row_step = 40;
  cloud.data.resize(80);
  std::copy(second.begin(), second.end(), cloud.data.begin() + 40);
  const auto filtered = filter_obstacle_height(cloud, identity(), 0, 0.8);
  EXPECT_EQ(filtered.width, 1U);
  EXPECT_EQ(filtered.height, 1U);
  EXPECT_EQ(filtered.data.size(), 32U);
}

TEST(ObstacleHeight, OppositeEndian)
{
  auto cloud = make_cloud({{0, 0, 0.2}, {0, 0, 1.0}});
  for (std::size_t offset = 0; offset < cloud.data.size(); offset += 8) {
    std::reverse(cloud.data.begin() + offset, cloud.data.begin() + offset + 8);
  }
  cloud.is_bigendian = !cloud.is_bigendian;
  const auto filtered = filter_obstacle_height(cloud, identity(), 0, 0.8);
  EXPECT_EQ(filtered.width, 1U);
  EXPECT_EQ(filtered.is_bigendian, cloud.is_bigendian);
  EXPECT_EQ(filtered.data, std::vector<uint8_t>(cloud.data.begin(), cloud.data.begin() + 32));
}

TEST(ObstacleHeight, EmptyCloudIsPublishedAsEmpty)
{
  const auto filtered = filter_obstacle_height(make_cloud({{0, 0, 1}}), identity(), 0, 0.8);
  EXPECT_EQ(filtered.width, 0U);
  EXPECT_TRUE(filtered.data.empty());
  EXPECT_EQ(filter_obstacle_height(make_cloud({}), identity(), 0, 0.8).width, 0U);
}

TEST(ObstacleHeight, InvalidInputsAreRejected)
{
  auto cloud = make_cloud({{0, 0, 0.5}});
  EXPECT_THROW(filter_obstacle_height(cloud, identity(), 1, 0), std::invalid_argument);
  EXPECT_THROW(filter_obstacle_height(cloud, identity(), 0,
      std::numeric_limits<double>::infinity()), std::invalid_argument);
  cloud.row_step = 1;
  EXPECT_THROW(filter_obstacle_height(cloud, identity(), 0, 0.8), std::invalid_argument);
  cloud = make_cloud({{0, 0, 0.5}});
  cloud.fields[2].name = "not_z";
  EXPECT_THROW(filter_obstacle_height(cloud, identity(), 0, 0.8), std::invalid_argument);
}
}  // namespace
