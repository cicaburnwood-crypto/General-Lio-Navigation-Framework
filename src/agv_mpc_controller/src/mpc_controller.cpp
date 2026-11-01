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

#include "agv_mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "speed_limit.hpp"

namespace agv_mpc_controller
{
namespace
{
template<typename T>
void declareAndGet(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const T & default_value,
  T & output)
{
  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(default_value));
  node->get_parameter(name, output);
}

double distance2(const Pose2D & a, const Pose2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

class CostmapObstacleQuery : public ObstacleQuery
{
public:
  CostmapObstacleQuery(
    nav2_costmap_2d::Costmap2D * costmap,
    const bool unknown_is_lethal,
    const double unknown_cost)
  : costmap_(costmap),
    unknown_is_lethal_(unknown_is_lethal),
    unknown_cost_(std::clamp(unknown_cost, 0.0, 1.0))
  {
  }

  bool inCollision(const double x, const double y, const double) const override
  {
    if (costmap_ == nullptr) {
      return true;
    }

    return cellCollision(x, y);
  }

  double normalizedCost(const double x, const double y) const override
  {
    if (costmap_ == nullptr) {
      return 1.0;
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(x, y, mx, my)) {
      return 1.0;
    }

    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      return unknown_is_lethal_ ? 1.0 : unknown_cost_;
    }
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return 1.0;
    }
    return static_cast<double>(cost) /
           static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }

private:
  bool cellCollision(const double x, const double y) const
  {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(x, y, mx, my)) {
      return true;
    }

    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      return unknown_is_lethal_;
    }
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  bool unknown_is_lethal_{false};
  double unknown_cost_{0.5};
};

}  // namespace

void MpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock parent node in agv_mpc_controller");
  }

  node_ = parent;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = std::move(name);
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  loadParameters(node);
  base_limits_ = config_.limits;
  mpc_.setConfig(config_);

  debug_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    name_ + "/selected_trajectory", rclcpp::QoS(1));

  RCLCPP_INFO(
    logger_,
    "Configured %s: horizon=%d dt=%.2f samples=(%d,%d,%d)",
    name_.c_str(), config_.horizon_steps, config_.time_step,
    config_.vx_samples, config_.vy_samples, config_.omega_samples);
}

void MpcController::cleanup()
{
  debug_pub_.reset();
  costmap_ros_.reset();
  tf_.reset();
}

void MpcController::activate()
{
  if (debug_pub_) {
    debug_pub_->on_activate();
  }
}

void MpcController::deactivate()
{
  if (debug_pub_) {
    debug_pub_->on_deactivate();
  }
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;

  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::PoseStamped robot_pose = pose;
  try {
    if (robot_pose.header.frame_id != global_frame) {
      robot_pose = tf_->transform(
        pose, global_frame, tf2::durationFromSec(transform_tolerance_));
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "%s failed to transform robot pose to %s: %s; commanding stop",
      name_.c_str(), global_frame.c_str(), ex.what());
    return makeZeroCommand(pose.header);
  }

  const auto plan = transformPlan(robot_pose);
  const auto header = robot_pose.header;
  if (plan.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "%s has no transformed plan; commanding stop", name_.c_str());
    return makeZeroCommand(header);
  }

  const Pose2D state = toPose2D(robot_pose);
  const Pose2D & goal = plan.back();
  if (std::sqrt(distance2(state, goal)) < config_.goal_tolerance &&
    std::fabs(normalizeAngle(goal.theta - state.theta)) < config_.yaw_goal_tolerance)
  {
    return makeZeroCommand(header);
  }

  const Velocity2D current_velocity{
    velocity.linear.x,
    velocity.linear.y,
    velocity.angular.z};

  CostmapObstacleQuery obstacles(
    costmap_ros_->getCostmap(), unknown_is_lethal_, unknown_cost_);
  const auto result = mpc_.optimize(state, current_velocity, plan, obstacles);

  if (!result.valid) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "%s found no valid MPC command (%s); commanding stop",
      name_.c_str(), result.reason.c_str());
    return makeZeroCommand(header);
  }

  publishDebugTrajectory(result, global_frame);
  return makeCommand(result.command, header);
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_ = path;
}

void MpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  config_.limits = detail::applySpeedLimit(base_limits_, speed_limit, percentage);
  mpc_.setConfig(config_);
}

void MpcController::loadParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  const auto p = [this](const std::string & param) {
      return name_ + "." + param;
    };

  declareAndGet(node, p("horizon_steps"), config_.horizon_steps, config_.horizon_steps);
  declareAndGet(node, p("time_step"), config_.time_step, config_.time_step);
  declareAndGet(node, p("vx_samples"), config_.vx_samples, config_.vx_samples);
  declareAndGet(node, p("vy_samples"), config_.vy_samples, config_.vy_samples);
  declareAndGet(node, p("omega_samples"), config_.omega_samples, config_.omega_samples);
  declareAndGet(node, p("allow_reverse"), config_.allow_reverse, config_.allow_reverse);
  declareAndGet(node, p("allow_lateral"), config_.allow_lateral, config_.allow_lateral);
  declareAndGet(node, p("robot_radius"), config_.robot_radius, config_.robot_radius);
  declareAndGet(node, p("goal_tolerance"), config_.goal_tolerance, config_.goal_tolerance);
  declareAndGet(
    node, p("yaw_goal_tolerance"), config_.yaw_goal_tolerance, config_.yaw_goal_tolerance);

  declareAndGet(node, p("min_vx"), config_.limits.min_vx, config_.limits.min_vx);
  declareAndGet(node, p("max_vx"), config_.limits.max_vx, config_.limits.max_vx);
  declareAndGet(node, p("min_vy"), config_.limits.min_vy, config_.limits.min_vy);
  declareAndGet(node, p("max_vy"), config_.limits.max_vy, config_.limits.max_vy);
  declareAndGet(node, p("min_omega"), config_.limits.min_omega, config_.limits.min_omega);
  declareAndGet(node, p("max_omega"), config_.limits.max_omega, config_.limits.max_omega);
  declareAndGet(
    node, p("max_accel_x"), config_.limits.max_accel_x, config_.limits.max_accel_x);
  declareAndGet(
    node, p("max_accel_y"), config_.limits.max_accel_y, config_.limits.max_accel_y);
  declareAndGet(
    node, p("max_accel_omega"), config_.limits.max_accel_omega,
    config_.limits.max_accel_omega);

  declareAndGet(node, p("path_weight"), config_.weights.path, config_.weights.path);
  declareAndGet(node, p("goal_weight"), config_.weights.goal, config_.weights.goal);
  declareAndGet(node, p("heading_weight"), config_.weights.heading, config_.weights.heading);
  declareAndGet(node, p("obstacle_weight"), config_.weights.obstacle, config_.weights.obstacle);
  declareAndGet(
    node, p("smoothness_weight"), config_.weights.smoothness, config_.weights.smoothness);
  declareAndGet(node, p("reverse_weight"), config_.weights.reverse, config_.weights.reverse);
  declareAndGet(node, p("lateral_weight"), config_.weights.lateral, config_.weights.lateral);
  declareAndGet(node, p("rotation_weight"), config_.weights.rotation, config_.weights.rotation);

  declareAndGet(node, p("transform_tolerance"), transform_tolerance_, transform_tolerance_);
  declareAndGet(node, p("plan_window_distance"), plan_window_distance_, plan_window_distance_);
  declareAndGet(node, p("debug_trajectory"), debug_trajectory_, debug_trajectory_);
  declareAndGet(node, p("unknown_is_lethal"), unknown_is_lethal_, unknown_is_lethal_);
  declareAndGet(node, p("unknown_cost"), unknown_cost_, unknown_cost_);

  config_.horizon_steps = std::max(1, config_.horizon_steps);
  config_.time_step = std::max(0.01, config_.time_step);
  config_.vx_samples = std::max(1, config_.vx_samples);
  config_.vy_samples = std::max(1, config_.vy_samples);
  config_.omega_samples = std::max(1, config_.omega_samples);
}

std::vector<Pose2D> MpcController::transformPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  nav_msgs::msg::Path plan;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    plan = global_plan_;
  }

  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  std::string plan_frame = plan.header.frame_id;
  if (plan_frame.empty() && !plan.poses.empty()) {
    plan_frame = plan.poses.front().header.frame_id;
  }
  if (plan_frame.empty()) {
    plan_frame = costmap_frame;
  }

  geometry_msgs::msg::PoseStamped robot_in_plan_frame = robot_pose;
  try {
    if (robot_in_plan_frame.header.frame_id != plan_frame) {
      robot_in_plan_frame = tf_->transform(
        robot_pose, plan_frame, tf2::durationFromSec(transform_tolerance_));
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Could not transform robot pose into plan frame %s: %s",
      plan_frame.c_str(), ex.what());
    return {};
  }

  const Pose2D robot = toPose2D(robot_pose);
  const Pose2D robot_for_search = toPose2D(robot_in_plan_frame);

  std::size_t closest = 0;
  double closest_dist = std::numeric_limits<double>::infinity();
  bool found_closest = false;
  for (std::size_t i = 0; i < plan.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped search_pose = plan.poses[i];
    if (search_pose.header.frame_id.empty()) {
      search_pose.header.frame_id = plan_frame;
    }

    try {
      if (search_pose.header.frame_id != plan_frame) {
        search_pose = tf_->transform(
          search_pose, plan_frame, tf2::durationFromSec(transform_tolerance_));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "Skipping untransformable plan pose: %s", ex.what());
      continue;
    }

    const double d2 = distance2(robot_for_search, toPose2D(search_pose));
    if (d2 < closest_dist) {
      closest_dist = d2;
      closest = i;
      found_closest = true;
    }
  }

  if (!found_closest) {
    return {};
  }

  std::vector<Pose2D> transformed;
  transformed.reserve(plan.poses.size() - closest);
  for (std::size_t i = closest; i < plan.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped transformed_pose = plan.poses[i];
    if (transformed_pose.header.frame_id.empty()) {
      transformed_pose.header.frame_id = plan_frame;
    }

    try {
      if (transformed_pose.header.frame_id != costmap_frame) {
        transformed_pose = tf_->transform(
          transformed_pose, costmap_frame, tf2::durationFromSec(transform_tolerance_));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "Skipping untransformable plan pose: %s", ex.what());
      continue;
    }

    const Pose2D point = toPose2D(transformed_pose);
    transformed.push_back(point);
    if (std::sqrt(distance2(robot, point)) > plan_window_distance_ &&
      transformed.size() >= 2)
    {
      break;
    }
  }

  return transformed;
}

Pose2D MpcController::toPose2D(const geometry_msgs::msg::PoseStamped & pose) const
{
  return Pose2D{
    pose.pose.position.x,
    pose.pose.position.y,
    tf2::getYaw(pose.pose.orientation)};
}

void MpcController::publishDebugTrajectory(
  const MpcResult & result,
  const std::string & frame_id)
{
  if (!debug_trajectory_ || !debug_pub_) {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = frame_id;
  path.poses.reserve(result.trajectory.size());

  for (const auto & point : result.trajectory) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.pose.x;
    pose.pose.position.y = point.pose.y;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, point.pose.theta);
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }

  debug_pub_->publish(path);
}

geometry_msgs::msg::TwistStamped MpcController::makeCommand(
  const Velocity2D & velocity,
  const std_msgs::msg::Header & header) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = header;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.twist.linear.x = velocity.vx;
  cmd.twist.linear.y = velocity.vy;
  cmd.twist.angular.z = velocity.omega;
  return cmd;
}

geometry_msgs::msg::TwistStamped MpcController::makeZeroCommand(
  const std_msgs::msg::Header & header) const
{
  return makeCommand(Velocity2D{}, header);
}

}  // namespace agv_mpc_controller

PLUGINLIB_EXPORT_CLASS(agv_mpc_controller::MpcController, nav2_core::Controller)
