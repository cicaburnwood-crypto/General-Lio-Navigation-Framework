#include "nav_goal_regenerator/table_goal_adapter.hpp"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace nav_goal_regenerator
{
namespace
{

constexpr double kTolerance = 1.0e-9;

OrientedTableConfig make_table()
{
  OrientedTableConfig config;
  config.length = 4.0;
  config.width = 2.0;
  config.waiting_offset = 0.33;
  config.centerline_tolerance = 0.10;
  return config;
}

}  // namespace

TEST(TableGoalAdapterTest, DetectsPointsInOrientedRectangle)
{
  auto config = make_table();
  config.center = {10.0, -2.0};
  config.yaw = 0.5 * std::acos(-1.0);
  TableGoalAdapter adapter(config);

  EXPECT_TRUE(adapter.contains({9.2, -0.5}));
  EXPECT_TRUE(adapter.contains({9.0, 0.0}));
  EXPECT_FALSE(adapter.contains({8.99, 0.0}));
  EXPECT_FALSE(adapter.contains({10.0, 0.01}));
}

TEST(TableGoalAdapterTest, PositiveShortAxisTargetUsesPositiveLongSide)
{
  TableGoalAdapter adapter(make_table());

  const auto result = adapter.adapt({1.5, 0.8}, {0.0, -5.0});

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->side, TableLongSide::kPositiveShortAxis);
  EXPECT_NEAR(result->waiting_point.x, 1.5, kTolerance);
  EXPECT_NEAR(result->waiting_point.y, 1.33, kTolerance);
  EXPECT_NEAR(result->inward_normal.x, 0.0, kTolerance);
  EXPECT_NEAR(result->inward_normal.y, -1.0, kTolerance);
  EXPECT_NEAR(result->waiting_yaw, -0.5 * std::acos(-1.0), kTolerance);
  EXPECT_NEAR(result->longitudinal_coordinate, 1.5, kTolerance);
}

TEST(TableGoalAdapterTest, NegativeShortAxisTargetUsesNegativeLongSide)
{
  TableGoalAdapter adapter(make_table());

  const auto result = adapter.adapt({-1.4, -0.7}, {0.0, 5.0});

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->side, TableLongSide::kNegativeShortAxis);
  EXPECT_NEAR(result->waiting_point.x, -1.4, kTolerance);
  EXPECT_NEAR(result->waiting_point.y, -1.33, kTolerance);
  EXPECT_NEAR(result->inward_normal.x, 0.0, kTolerance);
  EXPECT_NEAR(result->inward_normal.y, 1.0, kTolerance);
  EXPECT_NEAR(result->waiting_yaw, 0.5 * std::acos(-1.0), kTolerance);
}

TEST(TableGoalAdapterTest, ShortEndAndCornerTargetsStillUseLongSide)
{
  TableGoalAdapter adapter(make_table());

  const auto upper_corner = adapter.adapt({2.0, 1.0}, {5.0, 5.0});
  const auto lower_short_end = adapter.adapt({-1.95, -0.2}, {-5.0, -5.0});

  ASSERT_TRUE(upper_corner.has_value());
  EXPECT_EQ(upper_corner->side, TableLongSide::kPositiveShortAxis);
  EXPECT_NEAR(upper_corner->waiting_point.x, 2.0, kTolerance);
  EXPECT_NEAR(upper_corner->waiting_point.y, 1.33, kTolerance);
  ASSERT_TRUE(lower_short_end.has_value());
  EXPECT_EQ(lower_short_end->side, TableLongSide::kNegativeShortAxis);
  EXPECT_NEAR(lower_short_end->waiting_point.x, -1.95, kTolerance);
  EXPECT_NEAR(lower_short_end->waiting_point.y, -1.33, kTolerance);
}

TEST(TableGoalAdapterTest, CenterlineUsesCloserWaitingPoint)
{
  TableGoalAdapter adapter(make_table());

  const auto result = adapter.adapt({0.75, 0.05}, {0.75, -3.0});

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->side, TableLongSide::kNegativeShortAxis);
  EXPECT_NEAR(result->waiting_point.x, 0.75, kTolerance);
  EXPECT_NEAR(result->waiting_point.y, -1.33, kTolerance);
}

TEST(TableGoalAdapterTest, CenterlineDistanceTieIsDeterministic)
{
  TableGoalAdapter adapter(make_table());

  const auto result = adapter.adapt({0.25, 0.0}, {0.25, 0.0});

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->side, TableLongSide::kPositiveShortAxis);
}

TEST(TableGoalAdapterTest, RotatesWaitingPointNormalAndYawIntoMapFrame)
{
  auto config = make_table();
  config.center = {10.0, -2.0};
  config.yaw = 0.5 * std::acos(-1.0);
  TableGoalAdapter adapter(config);

  const auto result = adapter.adapt({9.2, -0.5}, {0.0, 0.0});

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->side, TableLongSide::kPositiveShortAxis);
  EXPECT_NEAR(result->waiting_point.x, 8.67, kTolerance);
  EXPECT_NEAR(result->waiting_point.y, -0.5, kTolerance);
  EXPECT_NEAR(result->inward_normal.x, 1.0, kTolerance);
  EXPECT_NEAR(result->inward_normal.y, 0.0, kTolerance);
  EXPECT_NEAR(result->waiting_yaw, 0.0, kTolerance);
  EXPECT_NEAR(result->longitudinal_coordinate, 1.5, kTolerance);
}

TEST(TableGoalAdapterTest, OutsideTargetAndInvalidInputsDoNotAdapt)
{
  auto config = make_table();
  TableGoalAdapter adapter(config);

  EXPECT_FALSE(adapter.adapt({2.01, 0.0}, {0.0, 0.0}).has_value());
  EXPECT_FALSE(adapter.adapt(
    {0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}).has_value());

  config.width = 0.0;
  adapter.setConfig(config);
  EXPECT_FALSE(adapter.isConfigValid());
  EXPECT_FALSE(adapter.adapt({0.0, 0.0}, {0.0, 0.0}).has_value());
}

}  // namespace nav_goal_regenerator
