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

#pragma once

#include <limits>
#include <string>
#include <vector>

namespace agv_mpc_controller
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct Velocity2D
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

struct TrajectoryPoint
{
  Pose2D pose;
  Velocity2D command;
  double time{0.0};
};

struct MpcLimits
{
  double min_vx{-0.1};
  double max_vx{0.3};
  double min_vy{-0.2};
  double max_vy{0.2};
  double min_omega{-0.5};
  double max_omega{0.5};
  double max_accel_x{0.6};
  double max_accel_y{0.6};
  double max_accel_omega{1.0};
};

struct MpcWeights
{
  double path{6.0};
  double goal{4.0};
  double heading{1.5};
  double obstacle{10.0};
  double smoothness{1.0};
  double reverse{0.5};
  double lateral{0.2};
  double rotation{0.1};
};

struct MpcConfig
{
  int horizon_steps{15};
  double time_step{0.1};
  int vx_samples{7};
  int vy_samples{5};
  int omega_samples{9};
  bool allow_reverse{true};
  bool allow_lateral{true};
  double robot_radius{0.35};
  double collision_cost{1.0};
  double goal_tolerance{0.10};
  double yaw_goal_tolerance{0.10};
  MpcLimits limits;
  MpcWeights weights;
};

class ObstacleQuery
{
public:
  virtual ~ObstacleQuery() = default;

  virtual bool inCollision(double x, double y, double radius) const = 0;
  virtual double normalizedCost(double x, double y) const = 0;
};

struct MpcResult
{
  bool valid{false};
  double cost{std::numeric_limits<double>::infinity()};
  Velocity2D command;
  std::vector<TrajectoryPoint> trajectory;
  std::string reason;
};

}  // namespace agv_mpc_controller
