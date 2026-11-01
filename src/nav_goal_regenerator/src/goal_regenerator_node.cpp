#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_goal_regenerator/goal_projector.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav_goal_regenerator
{
namespace
{

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  quaternion.normalize();
  return tf2::toMsg(quaternion);
}

}  // namespace

class GoalRegeneratorNode : public rclcpp::Node
{
public:
  GoalRegeneratorNode()
  : Node("goal_regenerator_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_goal_topic_ = declare_parameter<std::string>("input_goal_topic", "/goal_pose");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    output_goal_topic_ =
      declare_parameter<std::string>("output_goal_topic", "/validated_goal_pose");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    fallback_to_raw_ = declare_parameter<bool>("fallback_to_raw", false);
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.2);

    ProjectorConfig config;
    config.clearance_radius = declare_parameter<double>("clearance_radius", 0.33);
    config.max_search_radius = declare_parameter<double>("max_search_radius", 3.0);
    config.free_threshold = declare_parameter<int>("free_threshold", 25);
    config.allow_unknown = declare_parameter<bool>("allow_unknown", false);

    validate_parameters(config);
    projector_.setConfig(config);

    validated_goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(output_goal_topic_, 10);

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&GoalRegeneratorNode::map_callback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_goal_topic_, 10,
      std::bind(&GoalRegeneratorNode::goal_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Goal regenerator ready: %s + %s -> %s in frame '%s'",
      input_goal_topic_.c_str(), map_topic_.c_str(), output_goal_topic_.c_str(),
      target_frame_.c_str());
  }

private:
  void validate_parameters(const ProjectorConfig & config) const
  {
    if (target_frame_.empty()) {
      throw std::invalid_argument("target_frame must not be empty");
    }
    if (config.clearance_radius < 0.0) {
      throw std::invalid_argument("clearance_radius must be non-negative");
    }
    if (config.max_search_radius < 0.0) {
      throw std::invalid_argument("max_search_radius must be non-negative");
    }
    if (config.free_threshold < 0 || config.free_threshold > 100) {
      throw std::invalid_argument("free_threshold must be in [0, 100]");
    }
    if (transform_tolerance_ < 0.0) {
      throw std::invalid_argument("transform_tolerance must be non-negative");
    }
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    GridMap map;
    map.width = msg->info.width;
    map.height = msg->info.height;
    map.resolution = msg->info.resolution;
    map.origin_x = msg->info.origin.position.x;
    map.origin_y = msg->info.origin.position.y;
    map.origin_yaw = tf2::getYaw(msg->info.origin.orientation);
    map.data = msg->data;

    if (!projector_.isMapValid(map)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid occupancy grid: %ux%u, resolution %.3f, data size %zu",
        map.width, map.height, map.resolution, map.data.size());
      return;
    }

    latest_map_ = std::move(map);
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!transform_goal_to_target(*msg, goal)) {
      return;
    }

    if (!latest_map_.has_value()) {
      if (fallback_to_raw_) {
        publish_raw_goal(goal, "no map has been received");
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping goal: no occupancy grid has been received on '%s'",
          map_topic_.c_str());
      }
      return;
    }

    const Point2D original{goal.pose.position.x, goal.pose.position.y};
    const auto projection = projector_.project(*latest_map_, original);
    if (!projection.success) {
      if (fallback_to_raw_) {
        publish_raw_goal(goal, projection.reason);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping goal at (%.3f, %.3f): %s",
          original.x, original.y, projection.reason.c_str());
      }
      return;
    }

    auto output = goal;
    output.header.frame_id = target_frame_;
    output.pose.position.x = projection.point.x;
    output.pose.position.y = projection.point.y;
    output.pose.position.z = 0.0;
    if (projection.regenerated) {
      output.pose.orientation = yaw_to_quaternion(projection.yaw);
      RCLCPP_INFO(
        get_logger(),
        "Regenerated goal from (%.3f, %.3f) to (%.3f, %.3f), yaw %.3f rad",
        original.x, original.y, projection.point.x, projection.point.y, projection.yaw);
    }

    validated_goal_pub_->publish(output);
  }

  bool transform_goal_to_target(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output)
  {
    auto source = input;
    if (source.header.frame_id.empty()) {
      source.header.frame_id = target_frame_;
    }

    if (source.header.frame_id == target_frame_) {
      output = source;
      return true;
    }

    try {
      const auto timeout =
        rclcpp::Duration::from_seconds(std::max(0.0, transform_tolerance_));
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, source.header.frame_id, source.header.stamp, timeout);
      tf2::doTransform(source, output, transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping goal: cannot transform from '%s' to '%s': %s",
        source.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }
  }

  void publish_raw_goal(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & reason)
  {
    auto output = goal;
    output.header.frame_id = target_frame_;
    output.pose.position.z = 0.0;
    validated_goal_pub_->publish(output);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Publishing raw goal because fallback_to_raw is enabled: %s",
      reason.c_str());
  }

  std::string input_goal_topic_;
  std::string map_topic_;
  std::string output_goal_topic_;
  std::string target_frame_;
  bool fallback_to_raw_{false};
  double transform_tolerance_{0.2};

  GoalProjector projector_;
  std::optional<GridMap> latest_map_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr validated_goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace nav_goal_regenerator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav_goal_regenerator::GoalRegeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
