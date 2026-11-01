#include "nav_goal_regenerator/goal_projector.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

namespace nav_goal_regenerator
{
namespace
{

GridMap make_map(
  const std::uint32_t width,
  const std::uint32_t height,
  const double resolution,
  const std::int8_t default_value = 0)
{
  GridMap map;
  map.width = width;
  map.height = height;
  map.resolution = resolution;
  map.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), default_value);
  return map;
}

void set_cell(GridMap & map, const int mx, const int my, const std::int8_t value)
{
  map.data[static_cast<std::size_t>(my) * static_cast<std::size_t>(map.width) +
           static_cast<std::size_t>(mx)] = value;
}

}  // namespace

TEST(GoalProjectorTest, KeepsFreeGoalUnchanged)
{
  auto map = make_map(5, 5, 1.0);
  GoalProjector projector({0.0, 3.0, 25, false});

  const auto result = projector.project(map, {1.25, 1.75});

  ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.regenerated);
  EXPECT_DOUBLE_EQ(result.point.x, 1.25);
  EXPECT_DOUBLE_EQ(result.point.y, 1.75);
}

TEST(GoalProjectorTest, RegeneratesOccupiedGoalToNearestNavigableCell)
{
  auto map = make_map(6, 5, 1.0, 100);
  set_cell(map, 3, 2, 0);
  GoalProjector projector({0.0, 3.0, 25, false});

  const auto result = projector.project(map, {2.5, 2.5});

  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.regenerated);
  EXPECT_DOUBLE_EQ(result.point.x, 3.5);
  EXPECT_DOUBLE_EQ(result.point.y, 2.5);
}

TEST(GoalProjectorTest, UnknownCellsAreNotNavigableByDefault)
{
  auto map = make_map(5, 5, 1.0, -1);
  GoalProjector projector({0.0, 2.0, 25, false});

  const auto result = projector.project(map, {2.5, 2.5});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "no_navigable_cell");
}

TEST(GoalProjectorTest, GoalOutsideMapFails)
{
  auto map = make_map(5, 5, 1.0);
  GoalProjector projector({0.0, 2.0, 25, false});

  const auto result = projector.project(map, {-0.1, 2.5});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "outside_map");
}

TEST(GoalProjectorTest, ClearanceRejectsCellsTooCloseToObstacles)
{
  auto map = make_map(6, 6, 1.0);
  set_cell(map, 2, 2, 100);
  GoalProjector projector({1.1, 3.0, 25, false});

  EXPECT_FALSE(projector.isCellNavigable(map, 3, 2));
  EXPECT_TRUE(projector.isCellNavigable(map, 4, 2));
}

TEST(GoalProjectorTest, RegeneratedYawPointsTowardOriginalGoal)
{
  auto map = make_map(6, 5, 1.0, 100);
  set_cell(map, 3, 2, 0);
  GoalProjector projector({0.0, 3.0, 25, false});

  const auto result = projector.project(map, {2.5, 2.5});

  ASSERT_TRUE(result.success);
  ASSERT_TRUE(result.regenerated);
  EXPECT_NEAR(result.yaw, std::acos(-1.0), 1.0e-9);
}

}  // namespace nav_goal_regenerator
