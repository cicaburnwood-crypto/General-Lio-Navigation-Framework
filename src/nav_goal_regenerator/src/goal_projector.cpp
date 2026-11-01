#include "nav_goal_regenerator/goal_projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace nav_goal_regenerator
{
namespace
{

struct SearchCell
{
  double distance{0.0};
  int mx{0};
  int my{0};
};

struct SearchCellGreater
{
  bool operator()(const SearchCell & lhs, const SearchCell & rhs) const
  {
    if (lhs.distance == rhs.distance) {
      if (lhs.my == rhs.my) {
        return lhs.mx > rhs.mx;
      }
      return lhs.my > rhs.my;
    }
    return lhs.distance > rhs.distance;
  }
};

constexpr int kNeighborOffsets[8][2] = {
  {-1, -1}, {0, -1}, {1, -1},
  {-1, 0},           {1, 0},
  {-1, 1},  {0, 1},  {1, 1},
};

std::size_t cell_index(const GridMap & map, const int mx, const int my)
{
  return static_cast<std::size_t>(my) * static_cast<std::size_t>(map.width) +
         static_cast<std::size_t>(mx);
}

}  // namespace

GoalProjector::GoalProjector(ProjectorConfig config)
: config_(std::move(config))
{
}

void GoalProjector::setConfig(const ProjectorConfig & config)
{
  config_ = config;
}

const ProjectorConfig & GoalProjector::config() const
{
  return config_;
}

ProjectionResult GoalProjector::project(const GridMap & map, const Point2D & original) const
{
  ProjectionResult result;
  result.point = original;

  if (!isMapValid(map)) {
    result.reason = "invalid_map";
    return result;
  }

  int start_mx = 0;
  int start_my = 0;
  if (!worldToCell(map, original, start_mx, start_my)) {
    result.reason = "outside_map";
    return result;
  }

  if (isCellNavigable(map, start_mx, start_my)) {
    result.success = true;
    result.regenerated = false;
    return result;
  }

  const auto max_search_radius = std::max(0.0, config_.max_search_radius);
  std::priority_queue<SearchCell, std::vector<SearchCell>, SearchCellGreater> queue;
  std::vector<bool> visited(
    static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height), false);

  queue.push({distanceToCellCenter(map, start_mx, start_my, original), start_mx, start_my});
  visited[cell_index(map, start_mx, start_my)] = true;

  while (!queue.empty()) {
    const auto current = queue.top();
    queue.pop();

    if (current.distance > max_search_radius) {
      break;
    }

    if (isCellNavigable(map, current.mx, current.my)) {
      result.success = true;
      result.regenerated = true;
      result.point = cellCenterToWorld(map, current.mx, current.my);
      result.yaw = std::atan2(original.y - result.point.y, original.x - result.point.x);
      return result;
    }

    for (const auto & offset : kNeighborOffsets) {
      const auto next_mx = current.mx + offset[0];
      const auto next_my = current.my + offset[1];
      if (!isCellInBounds(map, next_mx, next_my)) {
        continue;
      }

      const auto index = cell_index(map, next_mx, next_my);
      if (visited[index]) {
        continue;
      }

      const auto distance = distanceToCellCenter(map, next_mx, next_my, original);
      if (distance <= max_search_radius) {
        queue.push({distance, next_mx, next_my});
      }
      visited[index] = true;
    }
  }

  result.reason = "no_navigable_cell";
  return result;
}

bool GoalProjector::isMapValid(const GridMap & map) const
{
  const auto expected_size =
    static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  return map.width > 0U && map.height > 0U && map.resolution > 0.0 &&
         map.data.size() == expected_size;
}

bool GoalProjector::worldToCell(
  const GridMap & map,
  const Point2D & point,
  int & mx,
  int & my) const
{
  if (!isMapValid(map)) {
    return false;
  }

  const auto dx = point.x - map.origin_x;
  const auto dy = point.y - map.origin_y;
  const auto cos_yaw = std::cos(map.origin_yaw);
  const auto sin_yaw = std::sin(map.origin_yaw);
  const auto local_x = cos_yaw * dx + sin_yaw * dy;
  const auto local_y = -sin_yaw * dx + cos_yaw * dy;

  mx = static_cast<int>(std::floor(local_x / map.resolution));
  my = static_cast<int>(std::floor(local_y / map.resolution));
  return isCellInBounds(map, mx, my);
}

Point2D GoalProjector::cellCenterToWorld(const GridMap & map, const int mx, const int my) const
{
  const auto local_x = (static_cast<double>(mx) + 0.5) * map.resolution;
  const auto local_y = (static_cast<double>(my) + 0.5) * map.resolution;
  const auto cos_yaw = std::cos(map.origin_yaw);
  const auto sin_yaw = std::sin(map.origin_yaw);

  return {
    map.origin_x + cos_yaw * local_x - sin_yaw * local_y,
    map.origin_y + sin_yaw * local_x + cos_yaw * local_y,
  };
}

bool GoalProjector::isCellNavigable(const GridMap & map, const int mx, const int my) const
{
  if (!isCellValueFree(map, mx, my)) {
    return false;
  }

  const auto clearance_radius = std::max(0.0, config_.clearance_radius);
  const auto cell_radius = static_cast<int>(std::ceil(clearance_radius / map.resolution));
  for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      const auto distance = std::hypot(
        static_cast<double>(dx) * map.resolution,
        static_cast<double>(dy) * map.resolution);
      if (distance > clearance_radius + std::numeric_limits<double>::epsilon()) {
        continue;
      }

      if (!isCellValueFree(map, mx + dx, my + dy)) {
        return false;
      }
    }
  }

  return true;
}

bool GoalProjector::isCellInBounds(const GridMap & map, const int mx, const int my) const
{
  return mx >= 0 && my >= 0 &&
         mx < static_cast<int>(map.width) &&
         my < static_cast<int>(map.height);
}

bool GoalProjector::isCellValueFree(const GridMap & map, const int mx, const int my) const
{
  if (!isMapValid(map) || !isCellInBounds(map, mx, my)) {
    return false;
  }

  const auto value = static_cast<int>(map.data[cell_index(map, mx, my)]);
  if (value < 0) {
    return config_.allow_unknown;
  }

  return value <= config_.free_threshold;
}

double GoalProjector::distanceToCellCenter(
  const GridMap & map,
  const int mx,
  const int my,
  const Point2D & point) const
{
  const auto center = cellCenterToWorld(map, mx, my);
  return std::hypot(center.x - point.x, center.y - point.y);
}

}  // namespace nav_goal_regenerator
