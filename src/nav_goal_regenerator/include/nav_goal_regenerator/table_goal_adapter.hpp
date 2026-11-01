#ifndef NAV_GOAL_REGENERATOR__TABLE_GOAL_ADAPTER_HPP_
#define NAV_GOAL_REGENERATOR__TABLE_GOAL_ADAPTER_HPP_

#include <optional>

#include "nav_goal_regenerator/goal_projector.hpp"

namespace nav_goal_regenerator
{

enum class TableLongSide
{
  kPositiveShortAxis,
  kNegativeShortAxis,
};

struct OrientedTableConfig
{
  Point2D center;
  double length{0.0};
  double width{0.0};
  double yaw{0.0};
  double waiting_offset{0.0};
  double centerline_tolerance{0.0};
};

struct TableApproach
{
  TableLongSide side{TableLongSide::kPositiveShortAxis};
  Point2D waiting_point;
  Point2D inward_normal;
  double waiting_yaw{0.0};
  double longitudinal_coordinate{0.0};
};

class TableGoalAdapter
{
public:
  explicit TableGoalAdapter(OrientedTableConfig config);

  void setConfig(const OrientedTableConfig & config);
  const OrientedTableConfig & config() const;

  bool isConfigValid() const;
  bool contains(const Point2D & point) const;

  // Returns an approach on one of the two long sides when target is inside the table.
  // If the target lies within centerline_tolerance, robot_position selects the closer side.
  std::optional<TableApproach> adapt(
    const Point2D & target,
    const Point2D & robot_position) const;

private:
  Point2D toLocal(const Point2D & point) const;
  Point2D toWorld(const Point2D & point) const;
  TableApproach makeApproach(
    double longitudinal_coordinate,
    TableLongSide side) const;

  OrientedTableConfig config_;
};

}  // namespace nav_goal_regenerator

#endif  // NAV_GOAL_REGENERATOR__TABLE_GOAL_ADAPTER_HPP_
