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

#include "agv_mpc_controller/sampling_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace agv_mpc_controller
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double sqr(double value)
{
  return value * value;
}

double distance2(const Pose2D & a, const Pose2D & b)
{
  return sqr(a.x - b.x) + sqr(a.y - b.y);
}

Pose2D integrate(const Pose2D & state, const Velocity2D & command, const double dt)
{
  Pose2D next = state;
  const double c = std::cos(state.theta);
  const double s = std::sin(state.theta);

  next.x += (command.vx * c - command.vy * s) * dt;
  next.y += (command.vx * s + command.vy * c) * dt;
  next.theta = normalizeAngle(next.theta + command.omega * dt);
  return next;
}

double clampAxis(
  const double sample,
  const double current,
  const double min_value,
  const double max_value,
  const double max_delta)
{
  const double lower_limit = std::min(min_value, max_value);
  const double upper_limit = std::max(min_value, max_value);
  const double delta = std::max(0.0, max_delta);
  const double lower = std::max(lower_limit, current - delta);
  const double upper = std::min(upper_limit, current + delta);

  if (lower > upper) {
    return std::clamp(sample, lower_limit, upper_limit);
  }

  return std::clamp(sample, lower, upper);
}

}  // namespace

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

SamplingMpc::SamplingMpc(MpcConfig config)
: config_(config)
{
}

void SamplingMpc::setConfig(const MpcConfig & config)
{
  config_ = config;
}

const MpcConfig & SamplingMpc::config() const
{
  return config_;
}

MpcResult SamplingMpc::optimize(
  const Pose2D & state,
  const Velocity2D & current_velocity,
  const std::vector<Pose2D> & reference_path,
  const ObstacleQuery & obstacles) const
{
  if (reference_path.empty()) {
    MpcResult result;
    result.reason = "empty reference path";
    return result;
  }

  const double min_vx = config_.allow_reverse ? config_.limits.min_vx :
    std::max(0.0, config_.limits.min_vx);
  const double max_vx = config_.allow_reverse ? config_.limits.max_vx :
    std::max(0.0, config_.limits.max_vx);
  const auto vx_samples = sampleAxis(min_vx, max_vx, config_.vx_samples);
  const auto vy_samples = config_.allow_lateral ?
    sampleAxis(config_.limits.min_vy, config_.limits.max_vy, config_.vy_samples) :
    std::vector<double>{0.0};
  const auto omega_samples =
    sampleAxis(config_.limits.min_omega, config_.limits.max_omega, config_.omega_samples);

  MpcResult best;
  best.reason = "no collision-free candidate";

  for (const double raw_vx : vx_samples) {
    for (const double raw_vy : vy_samples) {
      for (const double raw_omega : omega_samples) {
        const Velocity2D sample{raw_vx, raw_vy, raw_omega};
        const auto command = applyAccelerationLimits(sample, current_velocity);
        auto result =
          evaluateCandidate(state, command, current_velocity, reference_path, obstacles);
        if (result.valid && result.cost < best.cost) {
          best = result;
        } else if (!best.valid && !result.valid && !result.reason.empty()) {
          best.reason = result.reason;
        }
      }
    }
  }

  return best;
}

std::vector<double> SamplingMpc::sampleAxis(
  const double min_value,
  const double max_value,
  const int samples) const
{
  const double lower = std::min(min_value, max_value);
  const double upper = std::max(min_value, max_value);

  if (samples <= 1 || std::fabs(upper - lower) < 1e-9) {
    return {std::clamp(0.0, lower, upper)};
  }

  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(samples + 1));
  const double step = (upper - lower) / static_cast<double>(samples - 1);
  for (int i = 0; i < samples; ++i) {
    values.push_back(lower + step * static_cast<double>(i));
  }

  if (lower < 0.0 && upper > 0.0) {
    values.push_back(0.0);
  }

  std::sort(values.begin(), values.end());
  values.erase(
    std::unique(
      values.begin(), values.end(),
      [](double a, double b) {return std::fabs(a - b) < 1e-9;}), values.end());
  return values;
}

Velocity2D SamplingMpc::applyAccelerationLimits(
  const Velocity2D & sample,
  const Velocity2D & current_velocity) const
{
  const double dt = std::max(1e-3, config_.time_step);
  const double min_vx = config_.allow_reverse ? config_.limits.min_vx :
    std::max(0.0, config_.limits.min_vx);
  const double max_vx = config_.allow_reverse ? config_.limits.max_vx :
    std::max(0.0, config_.limits.max_vx);
  const double min_vy = config_.allow_lateral ? config_.limits.min_vy : 0.0;
  const double max_vy = config_.allow_lateral ? config_.limits.max_vy : 0.0;

  return Velocity2D{
    clampAxis(
      sample.vx, current_velocity.vx, min_vx, max_vx,
      config_.limits.max_accel_x * dt),
    clampAxis(
      sample.vy, current_velocity.vy, min_vy, max_vy,
      config_.limits.max_accel_y * dt),
    clampAxis(
      sample.omega, current_velocity.omega, config_.limits.min_omega,
      config_.limits.max_omega, config_.limits.max_accel_omega * dt)};
}

MpcResult SamplingMpc::evaluateCandidate(
  const Pose2D & state,
  const Velocity2D & command,
  const Velocity2D & current_velocity,
  const std::vector<Pose2D> & reference_path,
  const ObstacleQuery & obstacles) const
{
  MpcResult result;
  result.command = command;
  result.trajectory.reserve(static_cast<std::size_t>(config_.horizon_steps));

  Pose2D pose = state;
  double cost = 0.0;

  for (int step = 0; step < config_.horizon_steps; ++step) {
    pose = integrate(pose, command, config_.time_step);
    if (obstacles.inCollision(pose.x, pose.y, config_.robot_radius)) {
      result.reason = "collision";
      return result;
    }

    const double obstacle_cost = obstacles.normalizedCost(pose.x, pose.y);
    cost += config_.weights.path * pathCost(pose, reference_path);
    cost += config_.weights.heading * headingCost(pose, reference_path);
    cost += config_.weights.obstacle * obstacle_cost * obstacle_cost;

    result.trajectory.push_back(
      TrajectoryPoint{pose, command, config_.time_step * static_cast<double>(step + 1)});
  }

  const Pose2D & goal = reference_path.back();
  cost += config_.weights.goal * std::sqrt(distance2(pose, goal));
  cost += config_.weights.heading * std::fabs(normalizeAngle(goal.theta - pose.theta));
  cost += config_.weights.smoothness *
    (sqr(command.vx - current_velocity.vx) +
    sqr(command.vy - current_velocity.vy) +
    sqr(command.omega - current_velocity.omega));
  cost += config_.weights.reverse * std::max(0.0, -command.vx);
  cost += config_.weights.lateral * std::fabs(command.vy);
  cost += config_.weights.rotation * std::fabs(command.omega);

  const double goal_distance = std::sqrt(distance2(state, goal));
  if (goal_distance < config_.goal_tolerance &&
    std::fabs(normalizeAngle(goal.theta - state.theta)) < config_.yaw_goal_tolerance)
  {
    cost += sqr(command.vx) + sqr(command.vy) + sqr(command.omega);
  }

  result.valid = true;
  result.cost = cost;
  result.reason = "ok";
  return result;
}

double SamplingMpc::pathCost(
  const Pose2D & pose,
  const std::vector<Pose2D> & reference_path) const
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & point : reference_path) {
    best = std::min(best, distance2(pose, point));
  }
  return std::sqrt(best);
}

double SamplingMpc::headingCost(
  const Pose2D & pose,
  const std::vector<Pose2D> & reference_path) const
{
  double best = std::numeric_limits<double>::infinity();
  const Pose2D * nearest = nullptr;
  for (const auto & point : reference_path) {
    const double d2 = distance2(pose, point);
    if (d2 < best) {
      best = d2;
      nearest = &point;
    }
  }

  if (nearest == nullptr) {
    return 0.0;
  }
  return std::fabs(normalizeAngle(nearest->theta - pose.theta));
}

}  // namespace agv_mpc_controller
