#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav_goal_regenerator/direct_motion.hpp"
#include "nav_goal_regenerator/goal_projector.hpp"
#include "nav_goal_regenerator/table_goal_adapter.hpp"

namespace nav_goal_regenerator
{
namespace
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

GridMap to_grid_map(const nav_msgs::msg::OccupancyGrid & msg)
{
  GridMap map;
  map.width = msg.info.width;
  map.height = msg.info.height;
  map.resolution = msg.info.resolution;
  map.origin_x = msg.info.origin.position.x;
  map.origin_y = msg.info.origin.position.y;
  map.origin_yaw = tf2::getYaw(msg.info.origin.orientation);
  map.data = msg.data;
  return map;
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

const char * side_name(const TableLongSide side)
{
  return side == TableLongSide::kPositiveShortAxis ? "positive" : "negative";
}

}  // namespace

class TaskNavigationBridgeNode : public rclcpp::Node
{
public:
  TaskNavigationBridgeNode()
  : Node("task_navigation_bridge"),
    table_adapter_(OrientedTableConfig{}),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_target_topic_ =
      declare_parameter<std::string>("input_target_topic", "/navigation/target_pose");
    input_goal_topic_ = declare_parameter<std::string>("input_goal_topic", "/goal_pose");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    arrived_topic_ =
      declare_parameter<std::string>("arrived_topic", "/navigation/arrived");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    action_name_ = declare_parameter<std::string>("action_name", "/navigate_to_pose");
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.2);
    action_server_timeout_ = declare_parameter<double>("action_server_timeout", 2.0);
    fallback_to_raw_ = declare_parameter<bool>("fallback_to_raw", false);

    ProjectorConfig projector_config;
    projector_config.clearance_radius = declare_parameter<double>("clearance_radius", 0.33);
    projector_config.max_search_radius = declare_parameter<double>("max_search_radius", 3.0);
    projector_config.free_threshold = declare_parameter<int>("free_threshold", 25);
    projector_config.allow_unknown = declare_parameter<bool>("allow_unknown", false);
    projector_.setConfig(projector_config);

    OrientedTableConfig table_config;
    table_config.center.x = declare_parameter<double>("table_center_x", 2.13);
    table_config.center.y = declare_parameter<double>("table_center_y", 0.03);
    table_config.yaw = declare_parameter<double>("table_yaw", 0.0);
    table_config.length = declare_parameter<double>("table_length", 2.05);
    table_config.width = declare_parameter<double>("table_width", 1.25);
    table_config.waiting_offset = declare_parameter<double>("staging_offset", 0.33);
    table_config.centerline_tolerance =
      declare_parameter<double>("table_centerline_tolerance", 0.05);
    table_adapter_.setConfig(table_config);
    enable_table_approach_ = declare_parameter<bool>("enable_table_approach", true);

    insertion_distance_ = declare_parameter<double>("insertion_distance", 0.20);
    insertion_speed_ = declare_parameter<double>("insertion_speed", 0.05);
    control_rate_ = declare_parameter<double>("control_rate", 20.0);
    zero_hold_duration_ = declare_parameter<double>("zero_hold_duration", 0.30);
    heading_kp_ = declare_parameter<double>("heading_kp", 1.5);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.30);
    arrived_repeat_count_ = declare_parameter<int>("arrived_repeat_count", 2);
    arrived_repeat_interval_ = declare_parameter<double>("arrived_repeat_interval", 0.10);

    validate_parameters(projector_config);

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&TaskNavigationBridgeNode::map_callback, this, std::placeholders::_1));

    add_target_subscription(input_target_topic_);
    if (input_goal_topic_ != input_target_topic_) {
      add_target_subscription(input_goal_topic_);
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TaskNavigationBridgeNode::odom_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
    arrived_pub_ = create_publisher<std_msgs::msg::Bool>(
      arrived_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_));
    control_timer_ = create_wall_timer(
      control_period, std::bind(&TaskNavigationBridgeNode::control_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Unified navigation bridge ready: inputs=[%s, %s], table=%s, insert=%.3f m at %.3f m/s",
      input_target_topic_.c_str(), input_goal_topic_.c_str(),
      enable_table_approach_ ? "enabled" : "disabled", insertion_distance_, insertion_speed_);
  }

private:
  enum class Phase
  {
    kIdle,
    kNavigating,
    kInserting,
    kDocked,
    kRetreating,
    kZeroHold,
  };

  enum class ZeroHoldNext
  {
    kDocked,
    kStartRetreat,
    kDispatchPending,
  };

  struct PreparedGoal
  {
    geometry_msgs::msg::PoseStamped requested_goal;
    geometry_msgs::msg::PoseStamped nav_goal;
    bool is_table_goal{false};
    TableApproach table_approach;
  };

  void validate_parameters(const ProjectorConfig & projector_config) const
  {
    if (input_target_topic_.empty() || input_goal_topic_.empty()) {
      throw std::invalid_argument("navigation input topics must not be empty");
    }
    if (map_topic_.empty() || odom_topic_.empty() || cmd_vel_topic_.empty() ||
      arrived_topic_.empty())
    {
      throw std::invalid_argument("map, odom, cmd_vel, and arrived topics must not be empty");
    }
    if (target_frame_.empty() || odom_frame_.empty() || robot_base_frame_.empty() ||
      action_name_.empty())
    {
      throw std::invalid_argument(
              "target frame, odom frame, robot base frame, and action name must not be empty");
    }
    if (projector_config.clearance_radius < 0.0 || projector_config.max_search_radius < 0.0) {
      throw std::invalid_argument("projector distances must be non-negative");
    }
    if (projector_config.free_threshold < 0 || projector_config.free_threshold > 100) {
      throw std::invalid_argument("free_threshold must be in [0, 100]");
    }
    if (transform_tolerance_ < 0.0 || action_server_timeout_ < 0.0) {
      throw std::invalid_argument("transform and action-server timeouts must be non-negative");
    }
    if (!table_adapter_.isConfigValid()) {
      throw std::invalid_argument("table geometry parameters are invalid");
    }
    if (insertion_distance_ <= 0.0 || insertion_speed_ <= 0.0 || control_rate_ <= 0.0) {
      throw std::invalid_argument("insertion distance, speed, and control rate must be positive");
    }
    if (zero_hold_duration_ < 0.0 || heading_kp_ < 0.0 || max_angular_speed_ < 0.0) {
      throw std::invalid_argument("zero hold and heading-control parameters must be non-negative");
    }
    if (arrived_repeat_count_ < 1 || arrived_repeat_interval_ < 0.0 ||
      (arrived_repeat_count_ > 1 && arrived_repeat_interval_ <= 0.0))
    {
      throw std::invalid_argument("arrival repeat parameters are invalid");
    }
  }

  void add_target_subscription(const std::string & topic)
  {
    target_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, 10,
      [this, topic](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_callback(*msg, topic);
      }));
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    auto map = to_grid_map(*msg);
    if (!projector_.isMapValid(map)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid occupancy grid: %ux%u, resolution %.3f, data size %zu",
        map.width, map.height, map.resolution, map.data.size());
      return;
    }
    latest_map_ = std::move(map);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = PlanarPose{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      tf2::getYaw(msg->pose.pose.orientation),
    };
  }

  void target_callback(
    const geometry_msgs::msg::PoseStamped & input,
    const std::string & source_topic)
  {
    // TF lookup and map projection below can block this single-threaded
    // executor briefly.  Stop any direct motion before doing that work so a
    // newly received goal never leaves the previous straight-line command
    // latched during validation.  Invalid goals simply let the existing
    // direct phase resume on the next control tick.
    if (phase_ == Phase::kInserting || phase_ == Phase::kRetreating ||
      phase_ == Phase::kZeroHold || phase_ == Phase::kDocked)
    {
      publish_zero();
    }

    const auto prepared = prepare_goal(input);
    if (!prepared.has_value()) {
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Accepted new task goal from %s%s", source_topic.c_str(),
      prepared->is_table_goal ? " (table approach)" : "");
    publish_arrived(false);
    pending_goal_ = *prepared;

    switch (phase_) {
      case Phase::kIdle:
        dispatch_pending_goal();
        break;
      case Phase::kNavigating:
        request_navigation_cancel();
        break;
      case Phase::kInserting:
      case Phase::kDocked:
        publish_zero();
        begin_zero_hold(ZeroHoldNext::kStartRetreat);
        break;
      case Phase::kRetreating:
        // Keep backing out and use only the newest pending target afterwards.
        break;
      case Phase::kZeroHold:
        if (zero_hold_next_ == ZeroHoldNext::kDocked) {
          zero_hold_next_ = ZeroHoldNext::kStartRetreat;
        }
        break;
    }
  }

  std::optional<PreparedGoal> prepare_goal(const geometry_msgs::msg::PoseStamped & input)
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!transform_goal_to_target(input, goal)) {
      return std::nullopt;
    }

    PreparedGoal prepared;
    prepared.requested_goal = goal;
    const Point2D original{goal.pose.position.x, goal.pose.position.y};

    if (enable_table_approach_ && table_adapter_.contains(original)) {
      Point2D robot_position;
      if (!get_robot_map_position(robot_position)) {
        RCLCPP_WARN(get_logger(), "Rejecting table goal: current robot pose in map is unavailable");
        return std::nullopt;
      }

      const auto approach = table_adapter_.adapt(original, robot_position);
      if (!approach.has_value()) {
        RCLCPP_WARN(get_logger(), "Rejecting table goal: table approach adaptation failed");
        return std::nullopt;
      }

      goal.pose.position.x = approach->waiting_point.x;
      goal.pose.position.y = approach->waiting_point.y;
      goal.pose.position.z = 0.0;
      goal.pose.orientation = yaw_to_quaternion(approach->waiting_yaw);
      goal.header.frame_id = target_frame_;
      goal.header.stamp = now();

      prepared.nav_goal = goal;
      prepared.is_table_goal = true;
      prepared.table_approach = *approach;
      RCLCPP_INFO(
        get_logger(),
        "Adapted table target (%.3f, %.3f) to %s long-side staging (%.3f, %.3f), yaw %.3f",
        original.x, original.y, side_name(approach->side), approach->waiting_point.x,
        approach->waiting_point.y, approach->waiting_yaw);
      return prepared;
    }

    if (!project_goal(goal)) {
      return std::nullopt;
    }
    goal.header.frame_id = target_frame_;
    goal.header.stamp = now();
    prepared.nav_goal = goal;
    return prepared;
  }

  bool transform_goal_to_target(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output)
  {
    auto source = input;
    if (source.header.frame_id.empty()) {
      source.header.frame_id = target_frame_;
      RCLCPP_WARN(
        get_logger(), "Received target with empty frame_id; treating it as '%s'",
        target_frame_.c_str());
    }

    if (source.header.frame_id == target_frame_) {
      output = source;
      return true;
    }

    try {
      const auto timeout = rclcpp::Duration::from_seconds(transform_tolerance_);
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, source.header.frame_id,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()), timeout);
      tf2::doTransform(source, output, transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "Rejecting target: cannot transform from '%s' to '%s': %s",
        source.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool get_robot_map_position(Point2D & position)
  {
    try {
      const auto timeout = rclcpp::Duration::from_seconds(transform_tolerance_);
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, robot_base_frame_,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()), timeout);
      position.x = transform.transform.translation.x;
      position.y = transform.transform.translation.y;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Cannot obtain current robot pose: %s", ex.what());
      return false;
    }
  }

  bool project_goal(geometry_msgs::msg::PoseStamped & goal)
  {
    if (!latest_map_.has_value()) {
      if (fallback_to_raw_) {
        RCLCPP_WARN(get_logger(), "No map received; sending raw non-table goal");
        return true;
      }
      RCLCPP_WARN(get_logger(), "Rejecting non-table goal: no occupancy grid received");
      return false;
    }

    const Point2D original{goal.pose.position.x, goal.pose.position.y};
    const auto projection = projector_.project(*latest_map_, original);
    if (!projection.success) {
      if (fallback_to_raw_) {
        RCLCPP_WARN(
          get_logger(), "Sending raw goal because projection failed: %s",
          projection.reason.c_str());
        return true;
      }
      RCLCPP_WARN(
        get_logger(), "Rejecting goal at (%.3f, %.3f): %s", original.x, original.y,
        projection.reason.c_str());
      return false;
    }

    goal.pose.position.x = projection.point.x;
    goal.pose.position.y = projection.point.y;
    goal.pose.position.z = 0.0;
    if (projection.regenerated) {
      goal.pose.orientation = yaw_to_quaternion(projection.yaw);
      RCLCPP_INFO(
        get_logger(), "Projected goal from (%.3f, %.3f) to (%.3f, %.3f), yaw %.3f",
        original.x, original.y, projection.point.x, projection.point.y, projection.yaw);
    }
    return true;
  }

  void dispatch_pending_goal()
  {
    if (!pending_goal_.has_value()) {
      phase_ = Phase::kIdle;
      return;
    }

    active_goal_ = std::move(pending_goal_);
    pending_goal_.reset();
    send_navigation_goal(active_goal_->nav_goal);
  }

  void send_navigation_goal(const geometry_msgs::msg::PoseStamped & goal)
  {
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(action_server_timeout_));
    if (!action_client_->wait_for_action_server(timeout)) {
      RCLCPP_WARN(get_logger(), "Navigation action server '%s' is unavailable", action_name_.c_str());
      active_goal_.reset();
      phase_ = Phase::kIdle;
      publish_arrived(false);
      return;
    }

    NavigateToPose::Goal action_goal;
    action_goal.pose = goal;

    const auto request_id = ++nav_request_counter_;
    active_nav_request_ = request_id;
    active_nav_handle_.reset();
    nav_cancel_requested_ = false;
    phase_ = Phase::kNavigating;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, request_id](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
        handle_navigation_goal_response(request_id, goal_handle);
      };
    options.result_callback =
      [this, request_id](const GoalHandleNavigateToPose::WrappedResult & result) {
        handle_navigation_result(request_id, result);
      };

    RCLCPP_INFO(
      get_logger(), "Sending Nav2 goal: x=%.3f, y=%.3f, yaw=%.3f",
      goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation));
    action_client_->async_send_goal(action_goal, options);
  }

  void request_navigation_cancel()
  {
    if (nav_cancel_requested_) {
      return;
    }
    nav_cancel_requested_ = true;
    if (active_nav_handle_) {
      cancel_navigation_handle(active_nav_handle_, "active goal preempted by a newer task");
    } else {
      RCLCPP_INFO(get_logger(), "Waiting for pending Nav2 goal response before canceling it");
    }
  }

  void cancel_navigation_handle(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle,
    const char * reason)
  {
    try {
      action_client_->async_cancel_goal(goal_handle);
      RCLCPP_INFO(get_logger(), "Requested Nav2 cancellation: %s", reason);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Failed to request Nav2 cancellation: %s", ex.what());
    }
  }

  void handle_navigation_goal_response(
    const std::uint64_t request_id,
    const GoalHandleNavigateToPose::SharedPtr & goal_handle)
  {
    if (request_id != active_nav_request_) {
      if (goal_handle) {
        cancel_navigation_handle(goal_handle, "stale goal response");
      }
      return;
    }

    if (!goal_handle) {
      RCLCPP_WARN(get_logger(), "Nav2 rejected the task goal");
      active_nav_handle_.reset();
      nav_cancel_requested_ = false;
      active_goal_.reset();
      phase_ = Phase::kIdle;
      if (pending_goal_) {
        dispatch_pending_goal();
      } else {
        publish_arrived(false);
      }
      return;
    }

    active_nav_handle_ = goal_handle;
    if (nav_cancel_requested_) {
      cancel_navigation_handle(goal_handle, "goal was preempted before acceptance completed");
    } else {
      RCLCPP_INFO(get_logger(), "Nav2 accepted the task goal");
    }
  }

  void handle_navigation_result(
    const std::uint64_t request_id,
    const GoalHandleNavigateToPose::WrappedResult & result)
  {
    if (request_id != active_nav_request_) {
      return;
    }

    active_nav_handle_.reset();
    nav_cancel_requested_ = false;

    if (pending_goal_) {
      RCLCPP_INFO(get_logger(), "Old Nav2 goal reached a terminal state; dispatching newest task");
      active_goal_.reset();
      phase_ = Phase::kIdle;
      dispatch_pending_goal();
      return;
    }

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(
        get_logger(), "Nav2 task did not succeed (result code %d)",
        static_cast<int>(result.code));
      active_goal_.reset();
      phase_ = Phase::kIdle;
      publish_arrived(false);
      return;
    }

    if (active_goal_ && active_goal_->is_table_goal) {
      begin_insertion();
      return;
    }

    RCLCPP_INFO(get_logger(), "Non-table navigation task succeeded");
    active_goal_.reset();
    phase_ = Phase::kIdle;
    publish_arrived(true);
  }

  void begin_insertion()
  {
    if (!latest_odom_) {
      RCLCPP_WARN(get_logger(), "Cannot start table insertion: no odometry has been received");
      active_goal_.reset();
      phase_ = Phase::kIdle;
      publish_zero();
      publish_arrived(false);
      return;
    }

    direct_origin_ = *latest_odom_;
    direct_heading_ = table_heading_in_odom();
    direct_origin_->yaw = direct_heading_;
    phase_ = Phase::kInserting;
    RCLCPP_INFO(
      get_logger(), "Starting direct table insertion: distance=%.3f m, heading=%.3f rad",
      insertion_distance_, direct_heading_);
  }

  double table_heading_in_odom()
  {
    if (!active_goal_ || !active_goal_->is_table_goal) {
      return latest_odom_->yaw;
    }

    if (target_frame_ == odom_frame_) {
      return active_goal_->table_approach.waiting_yaw;
    }

    try {
      const auto timeout = rclcpp::Duration::from_seconds(transform_tolerance_);
      const auto transform = tf_buffer_.lookupTransform(
        odom_frame_, target_frame_,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()), timeout);
      const auto heading =
        tf2::getYaw(transform.transform.rotation) +
        active_goal_->table_approach.waiting_yaw;
      return std::atan2(std::sin(heading), std::cos(heading));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Cannot transform table normal into odom; holding current odom yaw instead: %s",
        ex.what());
      return latest_odom_->yaw;
    }
  }

  void control_tick()
  {
    publish_pending_arrival_repeat();

    switch (phase_) {
      case Phase::kInserting:
        if (direct_progress() >= insertion_distance_) {
          publish_zero();
          begin_zero_hold(ZeroHoldNext::kDocked);
        } else {
          publish_direct_command(1.0);
        }
        break;
      case Phase::kRetreating:
        if (direct_progress() <= 0.0) {
          publish_zero();
          begin_zero_hold(ZeroHoldNext::kDispatchPending);
        } else {
          publish_direct_command(-1.0);
        }
        break;
      case Phase::kZeroHold:
        publish_zero();
        if (zero_hold_elapsed()) {
          complete_zero_hold();
        }
        break;
      case Phase::kIdle:
      case Phase::kNavigating:
      case Phase::kDocked:
        break;
    }
  }

  double direct_progress() const
  {
    if (!latest_odom_ || !direct_origin_) {
      return 0.0;
    }
    return projected_progress(*direct_origin_, *latest_odom_);
  }

  void publish_direct_command(const double direction)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = direction * insertion_speed_;
    if (latest_odom_) {
      command.angular.z = heading_command(
        direct_heading_, latest_odom_->yaw, heading_kp_, max_angular_speed_);
    }
    cmd_vel_pub_->publish(command);
  }

  void publish_zero()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void begin_zero_hold(const ZeroHoldNext next)
  {
    zero_hold_next_ = next;
    zero_hold_started_ = std::chrono::steady_clock::now();
    phase_ = Phase::kZeroHold;
  }

  bool zero_hold_elapsed() const
  {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - zero_hold_started_).count();
    return elapsed >= zero_hold_duration_;
  }

  void complete_zero_hold()
  {
    switch (zero_hold_next_) {
      case ZeroHoldNext::kDocked:
        phase_ = Phase::kDocked;
        RCLCPP_INFO(
          get_logger(), "Table insertion reached %.3f m; task arrived", insertion_distance_);
        publish_arrived(true);
        break;
      case ZeroHoldNext::kStartRetreat:
        phase_ = Phase::kRetreating;
        RCLCPP_INFO(
          get_logger(), "Starting direct retreat from progress %.3f m", direct_progress());
        break;
      case ZeroHoldNext::kDispatchPending:
        direct_origin_.reset();
        active_goal_.reset();
        phase_ = Phase::kIdle;
        RCLCPP_INFO(get_logger(), "Direct retreat completed");
        dispatch_pending_goal();
        break;
    }
  }

  void publish_arrived(const bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    arrived_pub_->publish(message);

    arrived_repeat_value_ = value;
    arrived_repeats_remaining_ = arrived_repeat_count_ - 1;
    next_arrived_repeat_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(arrived_repeat_interval_));
  }

  void publish_pending_arrival_repeat()
  {
    if (arrived_repeats_remaining_ <= 0 ||
      std::chrono::steady_clock::now() < next_arrived_repeat_)
    {
      return;
    }

    std_msgs::msg::Bool message;
    message.data = arrived_repeat_value_;
    arrived_pub_->publish(message);
    --arrived_repeats_remaining_;
    next_arrived_repeat_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(arrived_repeat_interval_));
  }

  std::string input_target_topic_;
  std::string input_goal_topic_;
  std::string map_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string arrived_topic_;
  std::string target_frame_;
  std::string odom_frame_;
  std::string robot_base_frame_;
  std::string action_name_;
  double transform_tolerance_{0.2};
  double action_server_timeout_{2.0};
  bool fallback_to_raw_{false};
  bool enable_table_approach_{true};

  double insertion_distance_{0.20};
  double insertion_speed_{0.05};
  double control_rate_{20.0};
  double zero_hold_duration_{0.30};
  double heading_kp_{1.5};
  double max_angular_speed_{0.30};
  int arrived_repeat_count_{2};
  double arrived_repeat_interval_{0.10};

  GoalProjector projector_;
  TableGoalAdapter table_adapter_;
  std::optional<GridMap> latest_map_;
  std::optional<PlanarPose> latest_odom_;
  std::optional<PlanarPose> direct_origin_;
  double direct_heading_{0.0};

  Phase phase_{Phase::kIdle};
  ZeroHoldNext zero_hold_next_{ZeroHoldNext::kDocked};
  std::chrono::steady_clock::time_point zero_hold_started_;
  std::chrono::steady_clock::time_point next_arrived_repeat_;
  bool arrived_repeat_value_{false};
  int arrived_repeats_remaining_{0};
  std::optional<PreparedGoal> active_goal_;
  std::optional<PreparedGoal> pending_goal_;

  std::uint64_t nav_request_counter_{0};
  std::uint64_t active_nav_request_{0};
  bool nav_cancel_requested_{false};
  GoalHandleNavigateToPose::SharedPtr active_nav_handle_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> target_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace nav_goal_regenerator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav_goal_regenerator::TaskNavigationBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
