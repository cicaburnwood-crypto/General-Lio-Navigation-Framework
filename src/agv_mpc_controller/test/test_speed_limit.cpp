// Copyright 2026 waney
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "speed_limit.hpp"

#include "gtest/gtest.h"

namespace agv_mpc_controller
{
namespace
{

MpcLimits baseLimits()
{
  MpcLimits limits;
  limits.min_vx = -0.2;
  limits.max_vx = 0.4;
  limits.min_vy = -0.3;
  limits.max_vy = 0.3;
  limits.min_omega = -1.0;
  limits.max_omega = 1.0;
  return limits;
}

}  // namespace

TEST(SpeedLimitTest, PercentageSpeedLimitUsesBaseLimits)
{
  const MpcLimits limits = baseLimits();

  const MpcLimits half_limits = detail::applySpeedLimit(limits, 50.0, true);
  EXPECT_NEAR(half_limits.max_vx, 0.2, 1e-9);
  EXPECT_NEAR(half_limits.min_vx, -0.1, 1e-9);
  EXPECT_NEAR(half_limits.max_omega, 0.5, 1e-9);

  const MpcLimits eighty_limits = detail::applySpeedLimit(limits, 80.0, true);
  EXPECT_NEAR(eighty_limits.max_vx, 0.32, 1e-9);
  EXPECT_NEAR(eighty_limits.min_vx, -0.16, 1e-9);
  EXPECT_NEAR(eighty_limits.max_omega, 0.8, 1e-9);

  const MpcLimits reset_limits = detail::applySpeedLimit(limits, 0.0, true);
  EXPECT_NEAR(reset_limits.max_vx, limits.max_vx, 1e-9);
  EXPECT_NEAR(reset_limits.min_vx, limits.min_vx, 1e-9);
  EXPECT_NEAR(reset_limits.max_omega, limits.max_omega, 1e-9);
}

TEST(SpeedLimitTest, AbsoluteSpeedLimitUsesBaseLinearLimits)
{
  const MpcLimits limits = baseLimits();

  const MpcLimits limited = detail::applySpeedLimit(limits, 0.15, false);

  EXPECT_NEAR(limited.max_vx, 0.15, 1e-9);
  EXPECT_NEAR(limited.min_vx, -0.15, 1e-9);
  EXPECT_NEAR(limited.max_vy, 0.15, 1e-9);
  EXPECT_NEAR(limited.min_vy, -0.15, 1e-9);
  EXPECT_NEAR(limited.max_omega, limits.max_omega, 1e-9);
}

}  // namespace agv_mpc_controller
