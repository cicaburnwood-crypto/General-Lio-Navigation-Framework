#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "chassis_3d_localization/correction_smoothing.hpp"

namespace
{

using chassis_3d_localization::blend_map_from_odom;

Eigen::Matrix4f make_pose(const float x, const float y, const float yaw)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  pose(0, 3) = x;
  pose(1, 3) = y;
  return pose;
}

float yaw_of(const Eigen::Matrix4f & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

constexpr float kRad = static_cast<float>(M_PI) / 180.0F;

}  // namespace

TEST(CorrectionSmoothing, AlphaOneReturnsTargetExactly)
{
  const auto current = make_pose(1.0F, 2.0F, 0.3F);
  const auto target = make_pose(-0.5F, 0.25F, -1.2F);
  EXPECT_TRUE(blend_map_from_odom(current, target, 1.0).isApprox(target));
}

TEST(CorrectionSmoothing, AlphaClampedAtZeroKeepsCurrent)
{
  const auto current = make_pose(1.0F, 2.0F, 0.3F);
  const auto target = make_pose(-0.5F, 0.25F, -1.2F);
  EXPECT_TRUE(blend_map_from_odom(current, target, 0.0).isApprox(current, 1e-5F));
  EXPECT_TRUE(blend_map_from_odom(current, target, -3.0).isApprox(current, 1e-5F));
}

TEST(CorrectionSmoothing, HalfBlendTakesMidpoint)
{
  const auto current = make_pose(0.0F, 0.0F, 0.0F);
  const auto target = make_pose(2.0F, 1.0F, 90.0F * kRad);
  const auto blended = blend_map_from_odom(current, target, 0.5);
  EXPECT_NEAR(blended(0, 3), 1.0F, 1e-4F);
  EXPECT_NEAR(blended(1, 3), 0.5F, 1e-4F);
  EXPECT_NEAR(yaw_of(blended), 45.0F * kRad, 1e-4F);
}

TEST(CorrectionSmoothing, TakesShortestPathAcrossPi)
{
  const auto current = make_pose(0.0F, 0.0F, 170.0F * kRad);
  const auto target = make_pose(0.0F, 0.0F, -170.0F * kRad);
  const auto blended = blend_map_from_odom(current, target, 0.5);
  // Midpoint of +170deg and -170deg along the short arc is +-180deg, not 0.
  EXPECT_NEAR(std::cos(yaw_of(blended)), -1.0F, 1e-4F);
}

TEST(CorrectionSmoothing, SmallCorrectionAbsorbedByAlphaFraction)
{
  // The stationary-twitch case: solver scatter of 2deg, alpha = 0.4.
  const auto current = make_pose(0.0F, 0.0F, 0.0F);
  const auto target = make_pose(0.0F, 0.0F, 2.0F * kRad);
  const auto blended = blend_map_from_odom(current, target, 0.4);
  EXPECT_NEAR(yaw_of(blended), 0.8F * kRad, 1e-5F);
}

TEST(CorrectionSmoothing, RotationStaysOrthonormal)
{
  const auto current = make_pose(0.4F, -1.0F, 0.9F);
  const auto target = make_pose(-0.2F, 3.0F, -2.8F);
  const auto blended = blend_map_from_odom(current, target, 0.37);
  const Eigen::Matrix3f rotation = blended.block<3, 3>(0, 0);
  EXPECT_TRUE(
    (rotation * rotation.transpose()).isApprox(Eigen::Matrix3f::Identity(), 1e-5F));
  EXPECT_NEAR(rotation.determinant(), 1.0F, 1e-5F);
}
