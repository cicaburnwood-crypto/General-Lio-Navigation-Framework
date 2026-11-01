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

#include <cstddef>
#include <vector>

#include "gtest/gtest.h"

#include "agv_mpc_controller/sampling_mpc.hpp"

namespace agv_mpc_controller
{
namespace
{

class FreeSpace : public ObstacleQuery
{
public:
  bool inCollision(double, double, double) const override
  {
    return false;
  }

  double normalizedCost(double, double) const override
  {
    return 0.0;
  }
};

class BlockingSpace : public ObstacleQuery
{
public:
  bool inCollision(double, double, double) const override
  {
    return true;
  }

  double normalizedCost(double, double) const override
  {
    return 1.0;
  }
};

MpcConfig straightLineConfig()
{
  MpcConfig config;
  config.horizon_steps = 5;
  config.time_step = 0.1;
  config.vx_samples = 2;
  config.vy_samples = 1;
  config.omega_samples = 1;
  config.allow_reverse = false;
  config.allow_lateral = false;
  config.limits.min_vx = 0.0;
  config.limits.max_vx = 0.3;
  config.limits.min_vy = 0.0;
  config.limits.max_vy = 0.0;
  config.limits.min_omega = 0.0;
  config.limits.max_omega = 0.0;
  config.limits.max_accel_x = 10.0;
  config.limits.max_accel_y = 10.0;
  config.limits.max_accel_omega = 10.0;
  config.weights.path = 1.0;
  config.weights.goal = 10.0;
  config.weights.heading = 0.0;
  config.weights.obstacle = 0.0;
  config.weights.smoothness = 0.0;
  config.weights.reverse = 0.0;
  config.weights.lateral = 0.0;
  config.weights.rotation = 0.0;
  return config;
}

}  // namespace

TEST(SamplingMpcTest, EmptyReferencePathIsInvalid)
{
  const SamplingMpc mpc;
  const FreeSpace obstacles;

  const auto result = mpc.optimize(Pose2D{}, Velocity2D{}, {}, obstacles);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "empty reference path");
}

TEST(SamplingMpcTest, BlockingObstacleIsInvalid)
{
  const SamplingMpc mpc(straightLineConfig());
  const BlockingSpace obstacles;
  const std::vector<Pose2D> path{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};

  const auto result = mpc.optimize(Pose2D{}, Velocity2D{}, path, obstacles);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "collision");
}

TEST(SamplingMpcTest, StraightReferenceProducesForwardCommand)
{
  const MpcConfig config = straightLineConfig();
  const SamplingMpc mpc(config);
  const FreeSpace obstacles;
  const std::vector<Pose2D> path{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};

  const auto result = mpc.optimize(Pose2D{}, Velocity2D{}, path, obstacles);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.command.vx, 0.0);
  EXPECT_NEAR(result.command.vy, 0.0, 1e-9);
  EXPECT_NEAR(result.command.omega, 0.0, 1e-9);
  EXPECT_EQ(result.trajectory.size(), static_cast<std::size_t>(config.horizon_steps));
}

TEST(SamplingMpcTest, CurrentVelocityAboveLimitStillReturnsLegalCommand)
{
  MpcConfig config = straightLineConfig();
  config.horizon_steps = 1;
  config.limits.max_vx = 0.2;
  config.limits.max_accel_x = 0.1;
  const SamplingMpc mpc(config);
  const FreeSpace obstacles;
  const std::vector<Pose2D> path{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const Velocity2D current_velocity{1.0, 0.0, 0.0};

  const auto result = mpc.optimize(Pose2D{}, current_velocity, path, obstacles);

  ASSERT_TRUE(result.valid);
  EXPECT_GE(result.command.vx, config.limits.min_vx);
  EXPECT_LE(result.command.vx, config.limits.max_vx);
}

}  // namespace agv_mpc_controller
