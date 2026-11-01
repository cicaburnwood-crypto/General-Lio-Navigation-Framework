#ifndef NAV_GOAL_REGENERATOR__GOAL_PROJECTOR_HPP_
#define NAV_GOAL_REGENERATOR__GOAL_PROJECTOR_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace nav_goal_regenerator
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct GridMap
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  std::vector<std::int8_t> data;
};

struct ProjectorConfig
{
  double clearance_radius{0.33};
  double max_search_radius{3.0};
  int free_threshold{25};
  bool allow_unknown{false};
};

struct ProjectionResult
{
  bool success{false};
  bool regenerated{false};
  Point2D point;
  double yaw{0.0};
  std::string reason;
};

class GoalProjector
{
public:
  explicit GoalProjector(ProjectorConfig config = {});

  void setConfig(const ProjectorConfig & config);
  const ProjectorConfig & config() const;

  ProjectionResult project(const GridMap & map, const Point2D & original) const;

  bool isMapValid(const GridMap & map) const;
  bool worldToCell(const GridMap & map, const Point2D & point, int & mx, int & my) const;
  Point2D cellCenterToWorld(const GridMap & map, int mx, int my) const;
  bool isCellNavigable(const GridMap & map, int mx, int my) const;

private:
  bool isCellInBounds(const GridMap & map, int mx, int my) const;
  bool isCellValueFree(const GridMap & map, int mx, int my) const;
  double distanceToCellCenter(const GridMap & map, int mx, int my, const Point2D & point) const;

  ProjectorConfig config_;
};

}  // namespace nav_goal_regenerator

#endif  // NAV_GOAL_REGENERATOR__GOAL_PROJECTOR_HPP_
