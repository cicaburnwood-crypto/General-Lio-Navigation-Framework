#include "nav_goal_regenerator/arrival_monitor_state.hpp"

#include <cstdint>
#include <optional>

#include "gtest/gtest.h"

namespace nav_goal_regenerator
{
namespace
{

using State = ArrivalMonitorState;

State::GoalId make_goal_id(const std::uint8_t value)
{
  State::GoalId goal_id{};
  goal_id[15] = value;
  return goal_id;
}

State::GoalStatus make_status(
  const State::GoalId & goal_id,
  const std::int8_t status,
  const std::int64_t stamp_nanoseconds)
{
  State::GoalStatus output;
  output.goal_id = goal_id;
  output.status = status;
  output.stamp_nanoseconds = stamp_nanoseconds;
  return output;
}

void expect_no_event(const std::optional<bool> & event)
{
  EXPECT_FALSE(event.has_value());
}

}  // namespace

TEST(ArrivalMonitorStateTest, TargetPoseResetThenSucceededPublishesTrue)
{
  State state;
  const auto goal_id = make_goal_id(1);

  const auto reset_event = state.onGoalReset();
  ASSERT_TRUE(reset_event.has_value());
  EXPECT_FALSE(*reset_event);

  expect_no_event(state.onStatusList({make_status(goal_id, State::kExecuting, 10)}));

  const auto arrived_event = state.onStatusList({make_status(goal_id, State::kSucceeded, 10)});

  ASSERT_TRUE(arrived_event.has_value());
  EXPECT_TRUE(*arrived_event);
  expect_no_event(state.onStatusList({make_status(goal_id, State::kSucceeded, 10)}));
}

TEST(ArrivalMonitorStateTest, RvizGoalResetThenSucceededPublishesTrue)
{
  State state;
  const auto goal_id = make_goal_id(2);

  const auto reset_event = state.onGoalReset();
  ASSERT_TRUE(reset_event.has_value());
  EXPECT_FALSE(*reset_event);

  expect_no_event(state.onStatusList({make_status(goal_id, State::kAccepted, 20)}));

  const auto arrived_event = state.onStatusList({make_status(goal_id, State::kSucceeded, 20)});

  ASSERT_TRUE(arrived_event.has_value());
  EXPECT_TRUE(*arrived_event);
}

TEST(ArrivalMonitorStateTest, ResetMakesOldSucceededIgnored)
{
  State state;
  const auto old_goal_id = make_goal_id(3);
  const auto new_goal_id = make_goal_id(4);

  expect_no_event(state.onStatusList({make_status(old_goal_id, State::kExecuting, 30)}));

  const auto reset_event = state.onGoalReset();
  ASSERT_TRUE(reset_event.has_value());
  EXPECT_FALSE(*reset_event);

  expect_no_event(state.onStatusList({make_status(old_goal_id, State::kSucceeded, 30)}));
  expect_no_event(state.onStatusList({make_status(new_goal_id, State::kExecuting, 40)}));

  const auto arrived_event = state.onStatusList({make_status(new_goal_id, State::kSucceeded, 40)});

  ASSERT_TRUE(arrived_event.has_value());
  EXPECT_TRUE(*arrived_event);
}

TEST(ArrivalMonitorStateTest, AbortedAndCanceledGoalsPublishFalse)
{
  {
    State state;
    const auto goal_id = make_goal_id(5);

    const auto reset_event = state.onGoalReset();
    ASSERT_TRUE(reset_event.has_value());
    ASSERT_FALSE(*reset_event);
    expect_no_event(state.onStatusList({make_status(goal_id, State::kExecuting, 50)}));

    const auto aborted_event = state.onStatusList({make_status(goal_id, State::kAborted, 50)});

    ASSERT_TRUE(aborted_event.has_value());
    EXPECT_FALSE(*aborted_event);
  }

  {
    State state;
    const auto goal_id = make_goal_id(6);

    const auto reset_event = state.onGoalReset();
    ASSERT_TRUE(reset_event.has_value());
    ASSERT_FALSE(*reset_event);
    expect_no_event(state.onStatusList({make_status(goal_id, State::kExecuting, 60)}));

    const auto canceled_event = state.onStatusList({make_status(goal_id, State::kCanceled, 60)});

    ASSERT_TRUE(canceled_event.has_value());
    EXPECT_FALSE(*canceled_event);
  }
}

TEST(ArrivalMonitorStateTest, StartupHistoricalSucceededDoesNotPublishTrue)
{
  State state;
  const auto old_goal_id = make_goal_id(7);
  const auto new_goal_id = make_goal_id(8);

  expect_no_event(state.onStatusList({make_status(old_goal_id, State::kSucceeded, 70)}));
  expect_no_event(state.onStatusList({make_status(old_goal_id, State::kSucceeded, 70)}));

  expect_no_event(state.onStatusList({make_status(new_goal_id, State::kExecuting, 80)}));

  const auto arrived_event = state.onStatusList({make_status(new_goal_id, State::kSucceeded, 80)});

  ASSERT_TRUE(arrived_event.has_value());
  EXPECT_TRUE(*arrived_event);
}

}  // namespace nav_goal_regenerator
