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

#include <algorithm>

#include "agv_mpc_controller/types.hpp"

namespace agv_mpc_controller
{
namespace detail
{

inline MpcLimits applySpeedLimit(
  const MpcLimits & base_limits,
  const double speed_limit,
  const bool percentage)
{
  MpcLimits limits = base_limits;

  if (speed_limit <= 0.0) {
    return limits;
  }

  if (percentage) {
    const double scale = std::clamp(speed_limit / 100.0, 0.0, 1.0);
    limits.max_vx = base_limits.max_vx * scale;
    limits.min_vx = base_limits.min_vx * scale;
    limits.max_vy = base_limits.max_vy * scale;
    limits.min_vy = base_limits.min_vy * scale;
    limits.max_omega = base_limits.max_omega * scale;
    limits.min_omega = base_limits.min_omega * scale;
  } else {
    limits.max_vx = std::min(base_limits.max_vx, speed_limit);
    limits.min_vx = std::max(base_limits.min_vx, -speed_limit);
    limits.max_vy = std::min(base_limits.max_vy, speed_limit);
    limits.min_vy = std::max(base_limits.min_vy, -speed_limit);
  }

  return limits;
}

}  // namespace detail
}  // namespace agv_mpc_controller
