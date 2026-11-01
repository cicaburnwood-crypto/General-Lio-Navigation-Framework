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

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

#include "agv_mpc_controller/sampling_mpc.hpp"

namespace agv_mpc_controller
{

class MpcController : public nav2_core::Controller
{
public:
  MpcController() = default;
  ~MpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void loadParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  std::vector<Pose2D> transformPlan(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  Pose2D toPose2D(const geometry_msgs::msg::PoseStamped & pose) const;
  void publishDebugTrajectory(const MpcResult & result, const std::string & frame_id);
  geometry_msgs::msg::TwistStamped makeCommand(
    const Velocity2D & velocity,
    const std_msgs::msg::Header & header) const;
  geometry_msgs::msg::TwistStamped makeZeroCommand(const std_msgs::msg::Header & header) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("agv_mpc_controller")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  SamplingMpc mpc_;
  MpcConfig config_;
  MpcLimits base_limits_;
  double transform_tolerance_{0.2};
  double plan_window_distance_{6.0};
  bool debug_trajectory_{true};
  bool unknown_is_lethal_{false};
  double unknown_cost_{0.5};

  nav_msgs::msg::Path global_plan_;
  mutable std::mutex plan_mutex_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr debug_pub_;
};

}  // namespace agv_mpc_controller
