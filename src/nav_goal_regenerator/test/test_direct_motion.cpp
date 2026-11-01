#include "nav_goal_regenerator/direct_motion.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace nav_goal_regenerator
{
namespace
{

constexpr double kTolerance = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

TEST(DirectMotionTest, ProjectedProgressReachesTwentyCentimeterThreshold)
{
  const PlanarPose origin{1.0, -2.0, 0.0};

  EXPECT_LT(projected_progress(origin, {1.199, -2.0, 0.0}), 0.20);
  EXPECT_NEAR(projected_progress(origin, {1.20, -2.0, 0.0}), 0.20, kTolerance);
  EXPECT_GT(projected_progress(origin, {1.201, -2.0, 0.0}), 0.20);
}

TEST(DirectMotionTest, LateralDisplacementDoesNotContributeToProgress)
{
  const PlanarPose origin{1.0, -2.0, 0.0};

  EXPECT_NEAR(projected_progress(origin, {1.15, 100.0, 1.0}), 0.15, kTolerance);
}

TEST(DirectMotionTest, ProjectedProgressFollowsRotatedOriginHeading)
{
  const PlanarPose origin{2.0, 3.0, 0.5 * kPi};

  EXPECT_NEAR(projected_progress(origin, {-20.0, 3.20, -1.0}), 0.20, kTolerance);
}

TEST(DirectMotionTest, ReturningTowardAndPastOriginIsNonPositive)
{
  const PlanarPose origin{2.0, 3.0, 0.25 * kPi};
  const auto heading_x = std::cos(origin.yaw);
  const auto heading_y = std::sin(origin.yaw);

  EXPECT_NEAR(projected_progress(origin, origin), 0.0, kTolerance);
  EXPECT_LT(
    projected_progress(
      origin,
      {origin.x - 0.05 * heading_x, origin.y - 0.05 * heading_y, origin.yaw}),
    0.0);
}

TEST(DirectMotionTest, HeadingCommandWrapsAcrossPositivePiBoundary)
{
  const auto command = heading_command(-kPi + 0.05, kPi - 0.05, 2.0, 1.0);

  EXPECT_NEAR(command, 0.20, kTolerance);
}

TEST(DirectMotionTest, HeadingCommandWrapsAcrossNegativePiBoundary)
{
  const auto command = heading_command(kPi - 0.05, -kPi + 0.05, 2.0, 1.0);

  EXPECT_NEAR(command, -0.20, kTolerance);
}

TEST(DirectMotionTest, HeadingCommandIsSymmetricallyClamped)
{
  EXPECT_NEAR(heading_command(1.0, 0.0, 2.0, 0.30), 0.30, kTolerance);
  EXPECT_NEAR(heading_command(-1.0, 0.0, 2.0, 0.30), -0.30, kTolerance);
  EXPECT_NEAR(heading_command(1.0, 0.0, 2.0, -0.30), 0.30, kTolerance);
}

}  // namespace nav_goal_regenerator
