#include "nav_goal_regenerator/table_goal_adapter.hpp"

#include <cmath>
#include <utility>

namespace nav_goal_regenerator
{
namespace
{

constexpr double kGeometryEpsilon = 1.0e-9;

bool is_finite(const Point2D & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double squared_distance(const Point2D & lhs, const Point2D & rhs)
{
  const auto dx = lhs.x - rhs.x;
  const auto dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

double normalized_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

TableGoalAdapter::TableGoalAdapter(OrientedTableConfig config)
: config_(std::move(config))
{
}

void TableGoalAdapter::setConfig(const OrientedTableConfig & config)
{
  config_ = config;
}

const OrientedTableConfig & TableGoalAdapter::config() const
{
  return config_;
}

bool TableGoalAdapter::isConfigValid() const
{
  return is_finite(config_.center) &&
         std::isfinite(config_.length) && config_.length > 0.0 &&
         std::isfinite(config_.width) && config_.width > 0.0 &&
         std::isfinite(config_.yaw) &&
         std::isfinite(config_.waiting_offset) && config_.waiting_offset >= 0.0 &&
         std::isfinite(config_.centerline_tolerance) && config_.centerline_tolerance >= 0.0;
}

bool TableGoalAdapter::contains(const Point2D & point) const
{
  if (!isConfigValid() || !is_finite(point)) {
    return false;
  }

  const auto local = toLocal(point);
  return std::abs(local.x) <= config_.length * 0.5 + kGeometryEpsilon &&
         std::abs(local.y) <= config_.width * 0.5 + kGeometryEpsilon;
}

std::optional<TableApproach> TableGoalAdapter::adapt(
  const Point2D & target,
  const Point2D & robot_position) const
{
  if (!contains(target) || !is_finite(robot_position)) {
    return std::nullopt;
  }

  const auto local_target = toLocal(target);
  TableLongSide side;
  if (std::abs(local_target.y) <= config_.centerline_tolerance) {
    const auto positive_approach = makeApproach(
      local_target.x, TableLongSide::kPositiveShortAxis);
    const auto negative_approach = makeApproach(
      local_target.x, TableLongSide::kNegativeShortAxis);
    side = squared_distance(robot_position, negative_approach.waiting_point) <
      squared_distance(robot_position, positive_approach.waiting_point) ?
      TableLongSide::kNegativeShortAxis : TableLongSide::kPositiveShortAxis;
  } else {
    side = local_target.y > 0.0 ?
      TableLongSide::kPositiveShortAxis : TableLongSide::kNegativeShortAxis;
  }

  return makeApproach(local_target.x, side);
}

Point2D TableGoalAdapter::toLocal(const Point2D & point) const
{
  const auto dx = point.x - config_.center.x;
  const auto dy = point.y - config_.center.y;
  const auto cos_yaw = std::cos(config_.yaw);
  const auto sin_yaw = std::sin(config_.yaw);
  return {
    cos_yaw * dx + sin_yaw * dy,
    -sin_yaw * dx + cos_yaw * dy,
  };
}

Point2D TableGoalAdapter::toWorld(const Point2D & point) const
{
  const auto cos_yaw = std::cos(config_.yaw);
  const auto sin_yaw = std::sin(config_.yaw);
  return {
    config_.center.x + cos_yaw * point.x - sin_yaw * point.y,
    config_.center.y + sin_yaw * point.x + cos_yaw * point.y,
  };
}

TableApproach TableGoalAdapter::makeApproach(
  const double longitudinal_coordinate,
  const TableLongSide side) const
{
  const auto positive_side = side == TableLongSide::kPositiveShortAxis;
  const auto local_side_sign = positive_side ? 1.0 : -1.0;
  const auto waiting_local_y =
    local_side_sign * (config_.width * 0.5 + config_.waiting_offset);
  const Point2D inward_normal_local{0.0, -local_side_sign};
  const auto inward_normal_world = toWorld(inward_normal_local);

  TableApproach approach;
  approach.side = side;
  approach.waiting_point = toWorld({longitudinal_coordinate, waiting_local_y});
  approach.inward_normal = {
    inward_normal_world.x - config_.center.x,
    inward_normal_world.y - config_.center.y,
  };
  approach.waiting_yaw = normalized_angle(
    config_.yaw + (positive_side ? -0.5 : 0.5) * std::acos(-1.0));
  approach.longitudinal_coordinate = longitudinal_coordinate;
  return approach;
}

}  // namespace nav_goal_regenerator
