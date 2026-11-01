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

#include <vector>

#include "agv_mpc_controller/types.hpp"

namespace agv_mpc_controller
{

class SamplingMpc
{
public:
  explicit SamplingMpc(MpcConfig config = MpcConfig{});

  void setConfig(const MpcConfig & config);
  const MpcConfig & config() const;

  MpcResult optimize(
    const Pose2D & state,
    const Velocity2D & current_velocity,
    const std::vector<Pose2D> & reference_path,
    const ObstacleQuery & obstacles) const;

private:
  std::vector<double> sampleAxis(double min_value, double max_value, int samples) const;
  Velocity2D applyAccelerationLimits(
    const Velocity2D & sample,
    const Velocity2D & current_velocity) const;
  MpcResult evaluateCandidate(
    const Pose2D & state,
    const Velocity2D & command,
    const Velocity2D & current_velocity,
    const std::vector<Pose2D> & reference_path,
    const ObstacleQuery & obstacles) const;

  double pathCost(const Pose2D & pose, const std::vector<Pose2D> & reference_path) const;
  double headingCost(const Pose2D & pose, const std::vector<Pose2D> & reference_path) const;

  MpcConfig config_;
};

double normalizeAngle(double angle);

}  // namespace agv_mpc_controller
