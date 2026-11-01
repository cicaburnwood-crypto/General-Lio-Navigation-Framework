#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "livox_self_filter/self_filter.hpp"

namespace
{

livox_ros_driver2::msg::CustomPoint make_point(
  const float x,
  const float y,
  const float z,
  const std::uint32_t offset_time,
  const std::uint8_t reflectivity,
  const std::uint8_t tag,
  const std::uint8_t line)
{
  livox_ros_driver2::msg::CustomPoint point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.offset_time = offset_time;
  point.reflectivity = reflectivity;
  point.tag = tag;
  point.line = line;
  return point;
}

}  // namespace

TEST(SelfFilter, DropsPointsInsideBaseLinkBoxes)
{
  livox_ros_driver2::msg::CustomMsg input;
  input.header.frame_id = "lidar_front";
  input.timebase = 42;
  input.lidar_id = 7;
  input.rsvd = {1, 2, 3};
  input.points.push_back(make_point(0.0F, 0.0F, 0.2F, 10, 100, 1, 2));
  input.points.push_back(make_point(1.0F, 0.0F, 0.2F, 11, 101, 3, 4));
  input.point_num = input.points.size();

  const auto transform =
    livox_self_filter::make_transform({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  const auto boxes =
    livox_self_filter::boxes_from_flat({-0.1, -0.1, 0.0, 0.1, 0.1, 0.5});

  const auto output = livox_self_filter::filter_message(input, transform, boxes);

  ASSERT_EQ(output.point_num, 1U);
  ASSERT_EQ(output.points.size(), 1U);
  EXPECT_EQ(output.header.frame_id, "lidar_front");
  EXPECT_EQ(output.timebase, input.timebase);
  EXPECT_EQ(output.lidar_id, input.lidar_id);
  EXPECT_EQ(output.rsvd, input.rsvd);
  EXPECT_FLOAT_EQ(output.points[0].x, 1.0F);
  EXPECT_EQ(output.points[0].offset_time, 11U);
  EXPECT_EQ(output.points[0].reflectivity, 101U);
  EXPECT_EQ(output.points[0].tag, 3U);
  EXPECT_EQ(output.points[0].line, 4U);
}

TEST(SelfFilter, UsesBaseFromLidarTransform)
{
  const auto transform =
    livox_self_filter::make_transform({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  const auto boxes =
    livox_self_filter::boxes_from_flat({0.9, -0.1, -0.1, 1.1, 0.1, 0.1});
  const auto point = make_point(0.0F, 0.0F, 0.0F, 1, 2, 3, 4);

  EXPECT_TRUE(livox_self_filter::point_inside_boxes(point, transform, boxes));
}

TEST(SelfFilter, RejectsMalformedBoxes)
{
  EXPECT_THROW(
    livox_self_filter::boxes_from_flat({0.0, 1.0, 2.0}),
    std::invalid_argument);
}
