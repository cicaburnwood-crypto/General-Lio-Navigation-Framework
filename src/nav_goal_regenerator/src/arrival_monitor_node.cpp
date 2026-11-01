#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_goal_regenerator/arrival_monitor_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace nav_goal_regenerator
{
namespace
{

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

std::int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(stamp.nanosec);
}

ArrivalMonitorState::GoalId to_goal_id(const unique_identifier_msgs::msg::UUID & uuid)
{
  ArrivalMonitorState::GoalId goal_id{};
  std::copy(uuid.uuid.begin(), uuid.uuid.end(), goal_id.begin());
  return goal_id;
}

ArrivalMonitorState::GoalStatus to_goal_status(const action_msgs::msg::GoalStatus & status)
{
  ArrivalMonitorState::GoalStatus output;
  output.goal_id = to_goal_id(status.goal_info.goal_id);
  output.status = status.status;
  output.stamp_nanoseconds = to_nanoseconds(status.goal_info.stamp);
  return output;
}

std::chrono::nanoseconds to_duration(const double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

}  // namespace

class ArrivalMonitorNode : public rclcpp::Node
{
public:
  ArrivalMonitorNode()
  : Node("arrival_monitor")
  {
    arrived_topic_ = declare_parameter<std::string>("arrived_topic", "/navigation/arrived");
    status_topic_ =
      declare_parameter<std::string>("status_topic", "/navigate_to_pose/_action/status");
    reset_goal_topics_ = declare_parameter<std::vector<std::string>>(
      "reset_goal_topics", {"/navigation/target_pose", "/goal_pose"});
    repeat_count_ = declare_parameter<int>("repeat_count", 2);
    repeat_interval_ = declare_parameter<double>("repeat_interval", 0.1);

    validate_parameters();

    arrived_pub_ = create_publisher<std_msgs::msg::Bool>(
      arrived_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

    status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      status_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      std::bind(&ArrivalMonitorNode::status_callback, this, std::placeholders::_1));

    for (const auto & topic : reset_goal_topics_) {
      reset_goal_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, 10,
        [this, topic](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          (void)msg;
          reset_goal_callback(topic);
        }));
    }

    RCLCPP_INFO(
      get_logger(),
      "Arrival monitor ready: status=%s, arrived=%s, repeat_count=%d, repeat_interval=%.3f",
      status_topic_.c_str(), arrived_topic_.c_str(), repeat_count_, repeat_interval_);
  }

private:
  void validate_parameters() const
  {
    if (arrived_topic_.empty()) {
      throw std::invalid_argument("arrived_topic must not be empty");
    }
    if (status_topic_.empty()) {
      throw std::invalid_argument("status_topic must not be empty");
    }
    if (reset_goal_topics_.empty()) {
      throw std::invalid_argument("reset_goal_topics must not be empty");
    }
    for (const auto & topic : reset_goal_topics_) {
      if (topic.empty()) {
        throw std::invalid_argument("reset_goal_topics entries must not be empty");
      }
    }
    if (repeat_count_ < 1) {
      throw std::invalid_argument("repeat_count must be at least 1");
    }
    if (repeat_interval_ < 0.0) {
      throw std::invalid_argument("repeat_interval must be non-negative");
    }
    if (repeat_count_ > 1 && repeat_interval_ <= 0.0) {
      throw std::invalid_argument("repeat_interval must be positive when repeat_count > 1");
    }
  }

  void reset_goal_callback(const std::string & topic)
  {
    const auto event = state_.onGoalReset();
    if (event.has_value()) {
      RCLCPP_INFO(
        get_logger(), "Navigation goal received on %s; resetting arrival signal", topic.c_str());
      publish_repeated(*event);
    }
  }

  void status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    std::vector<ArrivalMonitorState::GoalStatus> statuses;
    statuses.reserve(msg->status_list.size());
    for (const auto & status : msg->status_list) {
      statuses.push_back(to_goal_status(status));
    }

    const auto event = state_.onStatusList(statuses);
    if (!event.has_value()) {
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Navigation goal finished; publishing arrived=%s",
      *event ? "true" : "false");
    publish_repeated(*event);
  }

  void publish_repeated(const bool arrived)
  {
    cancel_repeat_timer();
    publish_arrived(arrived);

    pending_value_ = arrived;
    pending_repeats_ = repeat_count_ - 1;
    if (pending_repeats_ <= 0) {
      return;
    }

    repeat_timer_ = create_wall_timer(
      to_duration(repeat_interval_),
      [this]() {
        publish_arrived(pending_value_);
        --pending_repeats_;
        if (pending_repeats_ <= 0) {
          cancel_repeat_timer();
        }
      });
  }

  void publish_arrived(const bool arrived)
  {
    std_msgs::msg::Bool msg;
    msg.data = arrived;
    arrived_pub_->publish(msg);
  }

  void cancel_repeat_timer()
  {
    if (repeat_timer_) {
      repeat_timer_->cancel();
      repeat_timer_.reset();
    }
    pending_repeats_ = 0;
  }

  std::string arrived_topic_;
  std::string status_topic_;
  std::vector<std::string> reset_goal_topics_;
  int repeat_count_{2};
  double repeat_interval_{0.1};

  ArrivalMonitorState state_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> reset_goal_subs_;
  rclcpp::TimerBase::SharedPtr repeat_timer_;
  bool pending_value_{false};
  int pending_repeats_{0};
};

}  // namespace nav_goal_regenerator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav_goal_regenerator::ArrivalMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
