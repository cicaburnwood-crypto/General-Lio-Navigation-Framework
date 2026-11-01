// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <memory>
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"

namespace mppi
{

namespace
{

visualization_msgs::msg::Marker createYawArrow(
  int id, const geometry_msgs::msg::Pose & pose, const std_msgs::msg::ColorRGBA & color,
  const std::string & frame_id, const std::string & marker_namespace, double length,
  double shaft_diameter, double head_diameter, double head_length)
{
  auto marker = utils::createMarker(
    id, pose, utils::createScale(shaft_diameter, head_diameter, head_length),
    color, frame_id, marker_namespace);
  marker.type = visualization_msgs::msg::Marker::ARROW;

  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;
  end.x = length;
  marker.points.push_back(start);
  marker.points.push_back(end);
  return marker;
}

}  // namespace

void TrajectoryVisualizer::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  const std::string & frame_id, ParametersHandler * parameters_handler)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  frame_id_ = frame_id;
  trajectories_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectories", 1);
  trajectory_yaws_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_yaws", 1);
  transformed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  parameters_handler_ = parameters_handler;

  auto getParam = parameters_handler->getParamGetter(name + ".TrajectoryVisualizer");

  getParam(trajectory_step_, "trajectory_step", 5);
  getParam(time_step_, "time_step", 3);

  reset();
}

void TrajectoryVisualizer::on_cleanup()
{
  trajectories_publisher_.reset();
  trajectory_yaws_publisher_.reset();
  transformed_path_pub_.reset();
}

void TrajectoryVisualizer::on_activate()
{
  trajectories_publisher_->on_activate();
  trajectory_yaws_publisher_->on_activate();
  transformed_path_pub_->on_activate();
}

void TrajectoryVisualizer::on_deactivate()
{
  trajectories_publisher_->on_deactivate();
  trajectory_yaws_publisher_->on_deactivate();
  transformed_path_pub_->on_deactivate();
}

void TrajectoryVisualizer::add(
  const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace)
{
  auto & size = trajectory.shape()[0];
  if (!size) {
    return;
  }

  auto add_marker = [&](auto i) {
      float component = static_cast<float>(i) / static_cast<float>(size);

      const bool has_yaw = trajectory.shape()[1] >= 3;
      auto pose = utils::createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
      auto scale =
        i != size - 1 ?
        utils::createScale(0.03, 0.03, 0.07) :
        utils::createScale(0.07, 0.07, 0.09);
      auto color = utils::createColor(0, component, component, 1);
      auto marker = utils::createMarker(
        marker_id_++, pose, scale, color, frame_id_, marker_namespace);
      points_->markers.push_back(marker);

      if (has_yaw) {
        auto yaw_pose =
          utils::createPose(trajectory(i, 0), trajectory(i, 1), 0.14, trajectory(i, 2));
        auto yaw_marker = createYawArrow(
          yaw_marker_id_++, yaw_pose, color, frame_id_, marker_namespace + " Yaw",
          0.03, 0.006, 0.016, 0.012);
        yaw_points_->markers.push_back(yaw_marker);
      }
    };

  for (size_t i = 0; i < size; i++) {
    add_marker(i);
  }
}

void TrajectoryVisualizer::add(
  const models::Trajectories & trajectories, const std::string & marker_namespace)
{
  auto & shape = trajectories.x.shape();
  const float shape_1 = static_cast<float>(shape[1]);
  points_->markers.reserve(floor(shape[0] / trajectory_step_) * floor(shape[1] * time_step_));
  yaw_points_->markers.reserve(floor(shape[0] / trajectory_step_) * floor(shape[1] * time_step_));

  for (size_t i = 0; i < shape[0]; i += trajectory_step_) {
    for (size_t j = 0; j < shape[1]; j += time_step_) {
      const float j_flt = static_cast<float>(j);
      float blue_component = 1.0f - j_flt / shape_1;
      float green_component = j_flt / shape_1;

      auto pose = utils::createPose(trajectories.x(i, j), trajectories.y(i, j), 0.03);
      auto scale = utils::createScale(0.03, 0.03, 0.03);
      auto color = utils::createColor(0, green_component, blue_component, 1);
      auto marker = utils::createMarker(
        marker_id_++, pose, scale, color, frame_id_, marker_namespace);

      points_->markers.push_back(marker);

      auto yaw_pose = utils::createPose(
        trajectories.x(i, j), trajectories.y(i, j), 0.10, trajectories.yaws(i, j));
      auto yaw_marker = createYawArrow(
        yaw_marker_id_++, yaw_pose, color, frame_id_, marker_namespace + " Yaw",
        0.018, 0.004, 0.012, 0.008);
      yaw_points_->markers.push_back(yaw_marker);
    }
  }
}

void TrajectoryVisualizer::reset()
{
  marker_id_ = 0;
  yaw_marker_id_ = 0;
  points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
  yaw_points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
}

void TrajectoryVisualizer::visualize(const nav_msgs::msg::Path & plan)
{
  if (trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }

  if (trajectory_yaws_publisher_->get_subscription_count() > 0) {
    trajectory_yaws_publisher_->publish(std::move(yaw_points_));
  }

  reset();

  if (transformed_path_pub_->get_subscription_count() > 0) {
    auto plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
    transformed_path_pub_->publish(std::move(plan_ptr));
  }
}

}  // namespace mppi
