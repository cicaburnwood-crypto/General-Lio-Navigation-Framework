#ifndef NAV_GOAL_REGENERATOR__ARRIVAL_MONITOR_STATE_HPP_
#define NAV_GOAL_REGENERATOR__ARRIVAL_MONITOR_STATE_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace nav_goal_regenerator
{

class ArrivalMonitorState
{
public:
  using GoalId = std::array<std::uint8_t, 16>;

  struct GoalStatus
  {
    GoalId goal_id{};
    std::int8_t status{0};
    std::int64_t stamp_nanoseconds{0};
  };

  static constexpr std::int8_t kAccepted = 1;
  static constexpr std::int8_t kExecuting = 2;
  static constexpr std::int8_t kCanceling = 3;
  static constexpr std::int8_t kSucceeded = 4;
  static constexpr std::int8_t kCanceled = 5;
  static constexpr std::int8_t kAborted = 6;

  std::optional<bool> onGoalReset()
  {
    for (const auto & goal_id : latest_active_goal_ids_) {
      ignore_goal(goal_id);
    }
    if (current_goal_id_.has_value()) {
      ignore_goal(*current_goal_id_);
    }
    current_goal_id_.reset();
    return false;
  }

  std::optional<bool> onStatusList(const std::vector<GoalStatus> & statuses)
  {
    const auto active_goal_ids = collect_active_goal_ids(statuses);
    const auto latest_active = find_latest_trackable_active_goal(statuses);

    if (!initialized_) {
      initialized_ = true;
      latest_active_goal_ids_ = active_goal_ids;
      if (latest_active.has_value()) {
        track_goal(*latest_active);
      }
      return std::nullopt;
    }

    if (latest_active.has_value()) {
      track_goal(*latest_active);
      latest_active_goal_ids_ = active_goal_ids;
      return std::nullopt;
    }

    if (current_goal_id_.has_value()) {
      const auto terminal_status = find_terminal_status(statuses, *current_goal_id_);
      if (terminal_status.has_value()) {
        const bool arrived = terminal_status->status == kSucceeded;
        const auto completed_goal_id = *current_goal_id_;
        current_goal_id_.reset();
        forget_ignored_goal(completed_goal_id);
        latest_active_goal_ids_ = active_goal_ids;
        return arrived;
      }
    }

    forget_terminal_ignored_goals(statuses);
    latest_active_goal_ids_ = active_goal_ids;
    return std::nullopt;
  }

private:
  static bool is_active_status(const std::int8_t status)
  {
    return status == kAccepted || status == kExecuting || status == kCanceling;
  }

  static bool is_terminal_status(const std::int8_t status)
  {
    return status == kSucceeded || status == kCanceled || status == kAborted;
  }

  static bool same_goal(const GoalId & lhs, const GoalId & rhs)
  {
    return lhs == rhs;
  }

  std::vector<GoalId> collect_active_goal_ids(const std::vector<GoalStatus> & statuses) const
  {
    std::vector<GoalId> active_goal_ids;
    for (const auto & status : statuses) {
      if (is_active_status(status.status)) {
        active_goal_ids.push_back(status.goal_id);
      }
    }
    return active_goal_ids;
  }

  std::optional<GoalStatus> find_latest_trackable_active_goal(
    const std::vector<GoalStatus> & statuses) const
  {
    std::optional<GoalStatus> latest;
    for (const auto & status : statuses) {
      if (!is_active_status(status.status) || is_ignored_goal(status.goal_id)) {
        continue;
      }
      if (!latest.has_value() || status.stamp_nanoseconds >= latest->stamp_nanoseconds) {
        latest = status;
      }
    }
    return latest;
  }

  std::optional<GoalStatus> find_terminal_status(
    const std::vector<GoalStatus> & statuses,
    const GoalId & goal_id) const
  {
    for (const auto & status : statuses) {
      if (same_goal(status.goal_id, goal_id) && is_terminal_status(status.status)) {
        return status;
      }
    }
    return std::nullopt;
  }

  void track_goal(const GoalStatus & status)
  {
    current_goal_id_ = status.goal_id;
  }

  void ignore_goal(const GoalId & goal_id)
  {
    if (!is_ignored_goal(goal_id)) {
      ignored_goal_ids_.push_back(goal_id);
    }
  }

  bool is_ignored_goal(const GoalId & goal_id) const
  {
    return std::find(ignored_goal_ids_.begin(), ignored_goal_ids_.end(), goal_id) !=
           ignored_goal_ids_.end();
  }

  void forget_ignored_goal(const GoalId & goal_id)
  {
    ignored_goal_ids_.erase(
      std::remove(ignored_goal_ids_.begin(), ignored_goal_ids_.end(), goal_id),
      ignored_goal_ids_.end());
  }

  void forget_terminal_ignored_goals(const std::vector<GoalStatus> & statuses)
  {
    for (const auto & status : statuses) {
      if (is_terminal_status(status.status)) {
        forget_ignored_goal(status.goal_id);
      }
    }
  }

  bool initialized_{false};
  std::optional<GoalId> current_goal_id_;
  std::vector<GoalId> latest_active_goal_ids_;
  std::vector<GoalId> ignored_goal_ids_;
};

}  // namespace nav_goal_regenerator

#endif  // NAV_GOAL_REGENERATOR__ARRIVAL_MONITOR_STATE_HPP_
