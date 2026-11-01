#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include "chassis_3d_localization/alignment_gate.hpp"
#include "chassis_3d_localization/correction_smoothing.hpp"
#include "chassis_3d_localization/score_gate.hpp"
#include "chassis_localization_msgs/srv/is_valid.hpp"
#include "chassis_localization_msgs/srv/relocalize.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "scan_registration/registration_methods.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

using CloudType = scan_registration::CloudType;
using PointType = scan_registration::PointType;
using Matrix4f = Eigen::Matrix4f;

struct LocalizerConfig
{
  std::string cloud_topic = "/sensors/cloud";
  std::string odom_topic = "/odom";
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  std::string pcd_map_path;
  std::string initialpose_topic = "/initialpose";
  std::string registration_method = "VGICP";
  double update_hz = 2.0;

  double submap_radius_xy = 12.0;
  double submap_radius_z = 3.0;

  double rough_scan_resolution = 0.25;
  double rough_map_resolution = 0.25;
  double refine_scan_resolution = 0.10;
  double refine_map_resolution = 0.10;

  int rough_max_iteration = 20;
  int refine_max_iteration = 30;
  double rough_max_correspondence_distance = 1.0;
  double refine_max_correspondence_distance = 0.5;
  // RMS correspondence error thresholds in meters. rough is a plain absolute
  // gate; refine_score_thresh is only the ScoreGate sanity ceiling - fine
  // filtering is relative to the running baseline (see score_gate.hpp).
  double rough_score_thresh = 0.4;
  double refine_score_thresh = 0.15;
  double refine_score_outlier_factor = 1.2;
  double score_baseline_beta = 0.2;
  int max_consecutive_score_rejections = 5;

  // Per-iteration convergence steps, split per stage: rough only seeds
  // refine (0.3 m correspondence basin) so it can stop early; refine's stop
  // criterion is the output precision so it stays tight.
  double rough_transformation_epsilon = 2.0e-3;
  double rough_rotation_epsilon = 1.0e-2;
  double refine_transformation_epsilon = 5.0e-4;
  double refine_rotation_epsilon = 2.0e-3;
  int correspondence_randomness = 20;
  int num_threads = 0;

  // EMA factor for blending accepted corrections into map->odom; 1.0
  // overwrites wholesale (disables smoothing).
  double correction_smoothing_alpha = 1.0;

  double max_translation_jump = 0.3;
  double max_yaw_jump_deg = 5.0;
  int max_consecutive_rejections = 5;
  int min_scan_points = 200;
  int min_submap_points = 1000;
  // map->odom is stamped now()+transform_tolerance so lookups between
  // alignment updates stay valid (same approach as AMCL).
  double transform_tolerance = 0.3;
  // Height band kept for registration, meters relative to base_link/map floor
  // plane. Drops ceiling points, which dominate small indoor scans but carry
  // no yaw information. Disabled when z_min >= z_max.
  double registration_z_min = -0.5;
  double registration_z_max = 2.2;
};

struct FrameData
{
  CloudType::Ptr cloud;
  Matrix4f odom_from_base = Matrix4f::Identity();
  builtin_interfaces::msg::Time stamp;
  std::uint64_t sequence = 0;
};

struct AlignmentAttempt
{
  bool succeeded = false;
  const char * decision = "unknown";
  std::size_t scan_points = 0;
  std::size_t submap_points = 0;
  double rough_rms = -1.0;
  double refine_rms = -1.0;
};

Matrix4f make_pose(
  const double x,
  const double y,
  const double z,
  const double yaw,
  const double pitch,
  const double roll)
{
  const Eigen::AngleAxisf yaw_angle(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
  const Eigen::AngleAxisf pitch_angle(static_cast<float>(pitch), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf roll_angle(static_cast<float>(roll), Eigen::Vector3f::UnitX());

  Matrix4f pose = Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  pose.block<3, 1>(0, 3) =
    Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
  return pose;
}

Matrix4f odom_to_matrix(const nav_msgs::msg::Odometry & odom)
{
  const Eigen::Quaternionf q(
    static_cast<float>(odom.pose.pose.orientation.w),
    static_cast<float>(odom.pose.pose.orientation.x),
    static_cast<float>(odom.pose.pose.orientation.y),
    static_cast<float>(odom.pose.pose.orientation.z));

  Matrix4f matrix = Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  matrix(0, 3) = static_cast<float>(odom.pose.pose.position.x);
  matrix(1, 3) = static_cast<float>(odom.pose.pose.position.y);
  matrix(2, 3) = static_cast<float>(odom.pose.pose.position.z);
  return matrix;
}

Matrix4f pose_to_matrix(const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Quaternionf q(
    static_cast<float>(pose.orientation.w),
    static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y),
    static_cast<float>(pose.orientation.z));

  Matrix4f matrix = Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  matrix(0, 3) = static_cast<float>(pose.position.x);
  matrix(1, 3) = static_cast<float>(pose.position.y);
  matrix(2, 3) = static_cast<float>(pose.position.z);
  return matrix;
}

geometry_msgs::msg::TransformStamped matrix_to_transform(
  const Matrix4f & matrix,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & parent,
  const std::string & child)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;

  const Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
  msg.transform.translation.x = matrix(0, 3);
  msg.transform.translation.y = matrix(1, 3);
  msg.transform.translation.z = matrix(2, 3);
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

void set_odom_pose(nav_msgs::msg::Odometry & odom, const Matrix4f & matrix)
{
  const Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
  odom.pose.pose.position.x = matrix(0, 3);
  odom.pose.pose.position.y = matrix(1, 3);
  odom.pose.pose.position.z = matrix(2, 3);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
}

double yaw_from_matrix(const Matrix4f & matrix)
{
  return std::atan2(matrix(1, 0), matrix(0, 0));
}

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

builtin_interfaces::msg::Time time_to_msg(const rclcpp::Time & time)
{
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  const auto nanoseconds = time.nanoseconds();

  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<int32_t>(nanoseconds / kNanosecondsPerSecond);
  msg.nanosec = static_cast<uint32_t>(nanoseconds % kNanosecondsPerSecond);
  return msg;
}

CloudType::Ptr voxel_filter(const CloudType::ConstPtr & cloud, const double resolution)
{
  CloudType::Ptr output(new CloudType);
  if (!cloud) {
    return output;
  }
  if (resolution <= 0.0) {
    *output = *cloud;
    return output;
  }

  pcl::VoxelGrid<PointType> filter;
  filter.setLeafSize(
    static_cast<float>(resolution),
    static_cast<float>(resolution),
    static_cast<float>(resolution));
  filter.setInputCloud(cloud);
  filter.filter(*output);
  return output;
}

CloudType::Ptr filter_z_band(
  const CloudType::ConstPtr & cloud,
  const double z_min,
  const double z_max)
{
  CloudType::Ptr output(new CloudType);
  if (!cloud) {
    return output;
  }
  if (z_min >= z_max) {
    *output = *cloud;
    return output;
  }

  output->reserve(cloud->size());
  for (const auto & point : cloud->points) {
    if (point.z < z_min || point.z > z_max) {
      continue;
    }
    output->push_back(point);
  }

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
  return output;
}

Eigen::Isometry3d matrix_to_isometry(const Matrix4f & matrix)
{
  return Eigen::Isometry3d(matrix.cast<double>());
}

CloudType::Ptr crop_submap(
  const CloudType::ConstPtr & map,
  const Eigen::Vector3f & center,
  const double radius_xy,
  const double radius_z)
{
  CloudType::Ptr submap(new CloudType);
  if (!map || map->empty()) {
    return submap;
  }

  const float radius_xy_f = static_cast<float>(std::max(0.0, radius_xy));
  const float radius_xy_sq = radius_xy_f * radius_xy_f;
  const float radius_z_f = static_cast<float>(radius_z);
  submap->reserve(map->size());

  for (const auto & point : map->points) {
    const float dx = point.x - center.x();
    const float dy = point.y - center.y();
    const float dz = point.z - center.z();
    if (dx * dx + dy * dy > radius_xy_sq) {
      continue;
    }
    if (radius_z > 0.0 && std::fabs(dz) > radius_z_f) {
      continue;
    }
    submap->push_back(point);
  }

  submap->width = static_cast<std::uint32_t>(submap->size());
  submap->height = 1;
  submap->is_dense = false;
  return submap;
}

}  // namespace

class Chassis3DLocalizer : public rclcpp::Node
{
public:
  Chassis3DLocalizer()
  : Node("chassis_3d_localizer_node")
  {
    load_parameters();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    localized_pub_ = create_publisher<nav_msgs::msg::Odometry>("localized_result", 10);
    map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_cloud", rclcpp::QoS(1).transient_local());

    relocalize_srv_ = create_service<chassis_localization_msgs::srv::Relocalize>(
      "relocalize",
      std::bind(
        &Chassis3DLocalizer::relocalize_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    relocalize_check_srv_ = create_service<chassis_localization_msgs::srv::IsValid>(
      "relocalize_check",
      std::bind(
        &Chassis3DLocalizer::relocalize_check_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      config_.initialpose_topic,
      10,
      std::bind(&Chassis3DLocalizer::initialpose_callback, this, std::placeholders::_1));

    const auto qos = rclcpp::QoS(10);
    cloud_sub_.subscribe(this, config_.cloud_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    odom_sub_.subscribe(this, config_.odom_topic, qos.get_rmw_qos_profile());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), cloud_sub_, odom_sub_);
    sync_->setAgePenalty(0.1);
    sync_->registerCallback(
      std::bind(
        &Chassis3DLocalizer::sync_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    chassis_3d_localization::AlignmentGate::Config gate_config;
    gate_config.max_translation_jump = config_.max_translation_jump;
    gate_config.max_yaw_jump_deg = config_.max_yaw_jump_deg;
    gate_config.max_consecutive_rejections = config_.max_consecutive_rejections;
    gate_ = chassis_3d_localization::AlignmentGate(gate_config);

    chassis_3d_localization::ScoreGate::Config score_config;
    score_config.ceiling = config_.refine_score_thresh;
    score_config.outlier_factor = config_.refine_score_outlier_factor;
    score_config.baseline_beta = config_.score_baseline_beta;
    score_config.max_consecutive_rejections = config_.max_consecutive_score_rejections;
    score_gate_ = chassis_3d_localization::ScoreGate(score_config);

    tf_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&Chassis3DLocalizer::tf_timer_callback, this));

    // Alignment runs in its own callback group so a slow VGICP solve cannot
    // stall TF publication; main spins a MultiThreadedExecutor.
    align_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto align_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / config_.update_hz));
    align_timer_ = create_wall_timer(
      align_period,
      std::bind(&Chassis3DLocalizer::align_timer_callback, this),
      align_callback_group_);

    if (!config_.pcd_map_path.empty()) {
      std::string message;
      if (!load_map(config_.pcd_map_path, message)) {
        throw std::runtime_error(message);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "3D localizer ready: cloud=%s, odom=%s, initialpose=%s, frames %s -> %s -> %s",
      config_.cloud_topic.c_str(),
      config_.odom_topic.c_str(),
      config_.initialpose_topic.c_str(),
      config_.map_frame.c_str(),
      config_.odom_frame.c_str(),
      config_.base_frame.c_str());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    nav_msgs::msg::Odometry>;

  void load_parameters()
  {
    config_.cloud_topic = declare_parameter<std::string>("cloud_topic", config_.cloud_topic);
    config_.odom_topic = declare_parameter<std::string>("odom_topic", config_.odom_topic);
    config_.map_frame = declare_parameter<std::string>("map_frame", config_.map_frame);
    config_.odom_frame = declare_parameter<std::string>("odom_frame", config_.odom_frame);
    config_.base_frame = declare_parameter<std::string>("base_frame", config_.base_frame);
    config_.pcd_map_path = declare_parameter<std::string>("pcd_map_path", config_.pcd_map_path);
    config_.initialpose_topic =
      declare_parameter<std::string>("initialpose_topic", config_.initialpose_topic);
    config_.registration_method =
      declare_parameter<std::string>("registration_method", config_.registration_method);
    config_.update_hz = declare_parameter<double>("update_hz", config_.update_hz);

    config_.submap_radius_xy =
      declare_parameter<double>("submap_radius_xy", config_.submap_radius_xy);
    config_.submap_radius_z =
      declare_parameter<double>("submap_radius_z", config_.submap_radius_z);

    config_.rough_scan_resolution =
      declare_parameter<double>("rough_scan_resolution", config_.rough_scan_resolution);
    config_.rough_map_resolution =
      declare_parameter<double>("rough_map_resolution", config_.rough_map_resolution);
    config_.refine_scan_resolution =
      declare_parameter<double>("refine_scan_resolution", config_.refine_scan_resolution);
    config_.refine_map_resolution =
      declare_parameter<double>("refine_map_resolution", config_.refine_map_resolution);

    config_.rough_max_iteration =
      declare_parameter<int>("rough_max_iteration", config_.rough_max_iteration);
    config_.refine_max_iteration =
      declare_parameter<int>("refine_max_iteration", config_.refine_max_iteration);
    config_.rough_max_correspondence_distance = declare_parameter<double>(
      "rough_max_correspondence_distance", config_.rough_max_correspondence_distance);
    config_.refine_max_correspondence_distance = declare_parameter<double>(
      "refine_max_correspondence_distance", config_.refine_max_correspondence_distance);
    config_.rough_score_thresh =
      declare_parameter<double>("rough_score_thresh", config_.rough_score_thresh);
    config_.refine_score_thresh =
      declare_parameter<double>("refine_score_thresh", config_.refine_score_thresh);
    config_.refine_score_outlier_factor = declare_parameter<double>(
      "refine_score_outlier_factor", config_.refine_score_outlier_factor);
    config_.score_baseline_beta =
      declare_parameter<double>("score_baseline_beta", config_.score_baseline_beta);
    config_.max_consecutive_score_rejections = declare_parameter<int>(
      "max_consecutive_score_rejections", config_.max_consecutive_score_rejections);

    config_.rough_transformation_epsilon = declare_parameter<double>(
      "rough_transformation_epsilon", config_.rough_transformation_epsilon);
    config_.rough_rotation_epsilon =
      declare_parameter<double>("rough_rotation_epsilon", config_.rough_rotation_epsilon);
    config_.refine_transformation_epsilon = declare_parameter<double>(
      "refine_transformation_epsilon", config_.refine_transformation_epsilon);
    config_.refine_rotation_epsilon =
      declare_parameter<double>("refine_rotation_epsilon", config_.refine_rotation_epsilon);
    config_.correspondence_randomness =
      declare_parameter<int>("correspondence_randomness", config_.correspondence_randomness);
    config_.num_threads = declare_parameter<int>("num_threads", config_.num_threads);
    config_.correction_smoothing_alpha = declare_parameter<double>(
      "correction_smoothing_alpha", config_.correction_smoothing_alpha);

    config_.max_translation_jump =
      declare_parameter<double>("max_translation_jump", config_.max_translation_jump);
    config_.max_yaw_jump_deg =
      declare_parameter<double>("max_yaw_jump_deg", config_.max_yaw_jump_deg);
    config_.max_consecutive_rejections =
      declare_parameter<int>("max_consecutive_rejections", config_.max_consecutive_rejections);
    config_.min_scan_points = declare_parameter<int>("min_scan_points", config_.min_scan_points);
    config_.min_submap_points =
      declare_parameter<int>("min_submap_points", config_.min_submap_points);
    config_.transform_tolerance =
      declare_parameter<double>("transform_tolerance", config_.transform_tolerance);
    config_.registration_z_min =
      declare_parameter<double>("registration_z_min", config_.registration_z_min);
    config_.registration_z_max =
      declare_parameter<double>("registration_z_max", config_.registration_z_max);

    if (config_.update_hz <= 0.0) {
      throw std::invalid_argument("update_hz must be positive");
    }
    if (config_.transform_tolerance < 0.0) {
      throw std::invalid_argument("transform_tolerance must be non-negative");
    }
    if (config_.rough_transformation_epsilon <= 0.0 ||
      config_.rough_rotation_epsilon <= 0.0 ||
      config_.refine_transformation_epsilon <= 0.0 ||
      config_.refine_rotation_epsilon <= 0.0)
    {
      throw std::invalid_argument("registration epsilons must be positive");
    }
    if (config_.refine_score_outlier_factor < 1.0) {
      throw std::invalid_argument("refine_score_outlier_factor must be >= 1.0");
    }
    if (config_.score_baseline_beta <= 0.0 || config_.score_baseline_beta > 1.0) {
      throw std::invalid_argument("score_baseline_beta must be in (0, 1]");
    }
    if (config_.correction_smoothing_alpha <= 0.0 ||
      config_.correction_smoothing_alpha > 1.0)
    {
      throw std::invalid_argument("correction_smoothing_alpha must be in (0, 1]");
    }
  }

  void sync_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
  {
    if (odom_msg->header.frame_id != config_.odom_frame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Expected odom frame '%s', got '%s'",
        config_.odom_frame.c_str(),
        odom_msg->header.frame_id.c_str());
    }
    if (!odom_msg->child_frame_id.empty() && odom_msg->child_frame_id != config_.base_frame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Expected odom child frame '%s', got '%s'",
        config_.base_frame.c_str(),
        odom_msg->child_frame_id.c_str());
    }

    CloudType::Ptr cloud(new CloudType);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_.cloud = cloud;
    latest_frame_.odom_from_base = odom_to_matrix(*odom_msg);
    latest_frame_.stamp = cloud_msg->header.stamp;
    ++latest_frame_.sequence;
    has_frame_ = true;
  }

  void initialpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != config_.map_frame) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring initial pose in frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(),
        config_.map_frame.c_str());
      return;
    }

    const auto initial_guess = pose_to_matrix(msg->pose.pose);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_loaded_) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring initial pose because no PCD map has been loaded yet");
        return;
      }

      pending_initial_guess_ = initial_guess;
      pending_initial_guess_available_ = true;
      localize_success_ = false;
      initialized_ = false;
      last_aligned_sequence_ = 0;
      ++state_epoch_;
      gate_.reset();
      score_gate_.reset();
    }

    RCLCPP_INFO(
      get_logger(),
      "Accepted RViz initial pose: x=%.3f y=%.3f z=%.3f yaw=%.3f",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z,
      yaw_from_matrix(initial_guess));
  }

  void tf_timer_callback()
  {
    Matrix4f map_from_odom = Matrix4f::Identity();
    Matrix4f odom_from_base = Matrix4f::Identity();
    builtin_interfaces::msg::Time stamp;
    bool has_frame = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!initialized_) {
        return;
      }
      map_from_odom = map_from_odom_;
      has_frame = has_frame_;
      odom_from_base = latest_frame_.odom_from_base;
      stamp = latest_frame_.stamp;
    }

    publish_tf(map_from_odom);
    if (has_frame) {
      publish_localized_result(stamp, map_from_odom * odom_from_base);
    }
  }

  void align_timer_callback()
  {
    FrameData frame;
    CloudType::Ptr rough_map;
    CloudType::Ptr refine_map;
    Matrix4f map_from_odom = Matrix4f::Identity();
    Matrix4f pending_initial_guess = Matrix4f::Identity();
    std::uint64_t last_aligned_sequence = 0;
    std::uint64_t state_epoch = 0;
    bool has_pending_initial_guess = false;
    bool initialized = false;
    bool map_loaded = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_frame_) {
        return;
      }
      frame = latest_frame_;
      rough_map = rough_map_;
      refine_map = refine_map_;
      map_from_odom = map_from_odom_;
      pending_initial_guess = pending_initial_guess_;
      last_aligned_sequence = last_aligned_sequence_;
      state_epoch = state_epoch_;
      has_pending_initial_guess = pending_initial_guess_available_;
      initialized = initialized_;
      map_loaded = map_loaded_;
    }

    if (!map_loaded || !rough_map || !refine_map) {
      return;
    }
    if (!has_pending_initial_guess && !initialized) {
      return;
    }
    if (frame.sequence == last_aligned_sequence) {
      return;
    }

    const Matrix4f initial_map_from_base =
      has_pending_initial_guess ? pending_initial_guess : map_from_odom * frame.odom_from_base;
    Matrix4f aligned_map_from_base = Matrix4f::Identity();

    auto attempt = align_scan_to_map(
      frame.cloud,
      rough_map,
      refine_map,
      initial_map_from_base,
      aligned_map_from_base);

    chassis_3d_localization::AlignmentGate::JumpMetrics jump;
    int consecutive_rejections = 0;
    double score_baseline = -1.0;
    Matrix4f new_map_from_odom = map_from_odom;
    bool apply_update = false;
    bool jump_overridden = false;
    bool score_overridden = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // A new initial pose / map arrived while this solve was running; its
      // reset must win, so discard this result instead of clobbering it.
      if (state_epoch_ != state_epoch) {
        attempt.succeeded = false;
        attempt.decision = "stale_state";
      } else {
        last_aligned_sequence_ = frame.sequence;

        if (attempt.succeeded) {
          // Quality first (is this solution trustworthy at all), consistency
          // second: the jump gate must only see score-approved solutions so
          // its override never admits a low-quality result. A score-accepted
          // but jump-rejected solution still refreshes the baseline - the
          // baseline tracks achievable quality, not applied poses.
          const auto score_decision = score_gate_.evaluate(attempt.refine_rms);
          if (score_decision ==
            chassis_3d_localization::ScoreGate::Decision::kRejectedCeiling)
          {
            attempt.decision = "reject_score_ceiling";
          } else if (score_decision ==
            chassis_3d_localization::ScoreGate::Decision::kRejectedOutlier)
          {
            attempt.decision = "reject_score_outlier";
          } else {
            score_overridden = score_decision ==
              chassis_3d_localization::ScoreGate::Decision::kScoreOverride;

            jump = compute_jump_metrics(initial_map_from_base, aligned_map_from_base);
            const bool bypass_jump_check = has_pending_initial_guess || !initialized;
            const auto decision = gate_.evaluate(jump, bypass_jump_check);

            if (decision == chassis_3d_localization::AlignmentGate::Decision::kRejectedJump) {
              attempt.decision = "reject_jump";
            } else {
              jump_overridden =
                decision == chassis_3d_localization::AlignmentGate::Decision::kJumpOverride;
              attempt.decision = jump_overridden ? "jump_override" :
                (score_overridden ? "score_override" : "accepted");

              const Matrix4f raw_map_from_odom =
                aligned_map_from_base * frame.odom_from_base.inverse();
              // Initialization and jump overrides snap to the solution;
              // tracking corrections are EMA-blended so per-solve scatter
              // does not twitch the pose (see correction_smoothing.hpp).
              const bool full_jump = bypass_jump_check || jump_overridden;
              new_map_from_odom = full_jump ? raw_map_from_odom :
                chassis_3d_localization::blend_map_from_odom(
                map_from_odom, raw_map_from_odom, config_.correction_smoothing_alpha);
              map_from_odom_ = new_map_from_odom;
              initialized_ = true;
              localize_success_ = true;
              pending_initial_guess_available_ = false;
              apply_update = true;
            }
          }
        }
        consecutive_rejections = gate_.consecutive_rejections();
        score_baseline = score_gate_.baseline();
      }
    }

    if (apply_update) {
      publish_tf(new_map_from_odom);
      publish_localized_result(frame.stamp, new_map_from_odom * frame.odom_from_base);
    }
    if (jump_overridden) {
      RCLCPP_WARN(
        get_logger(),
        "Jump gate override after repeated rejections: translation=%.3f m yaw=%.2f deg",
        jump.translation,
        jump.yaw_deg);
    }
    if (score_overridden) {
      RCLCPP_WARN(
        get_logger(),
        "Score gate override: refine_rms=%.3f exceeded the baseline outlier cut "
        "repeatedly; baseline re-seeded (local scene/floor changed?)",
        attempt.refine_rms);
    }

    const Matrix4f & log_map_from_odom = apply_update ? new_map_from_odom : map_from_odom;
    // jump_* stay raw (solution vs prediction, unaffected by smoothing) so
    // solver scatter remains observable; map_odom shows the blended output.
    RCLCPP_INFO(
      get_logger(),
      "[align] seq=%llu scan=%zu submap=%zu rough_rms=%.3f refine_rms=%.3f "
      "jump_t=%.3fm jump_yaw=%.2fdeg consec_rej=%d decision=%s map_odom=(%.3f, %.3f, %.1fdeg) "
      "score_base=%.3f",
      static_cast<unsigned long long>(frame.sequence),
      attempt.scan_points,
      attempt.submap_points,
      attempt.rough_rms,
      attempt.refine_rms,
      jump.translation,
      jump.yaw_deg,
      consecutive_rejections,
      attempt.decision,
      static_cast<double>(log_map_from_odom(0, 3)),
      static_cast<double>(log_map_from_odom(1, 3)),
      yaw_from_matrix(log_map_from_odom) * 180.0 / M_PI,
      score_baseline);
  }

  AlignmentAttempt align_scan_to_map(
    const CloudType::ConstPtr & scan,
    const CloudType::ConstPtr & rough_map,
    const CloudType::ConstPtr & refine_map,
    const Matrix4f & initial_map_from_base,
    Matrix4f & aligned_map_from_base)
  {
    AlignmentAttempt attempt;
    attempt.scan_points = scan ? scan->size() : 0;

    if (!scan || scan->size() < static_cast<std::size_t>(config_.min_scan_points)) {
      attempt.decision = "skip_small_scan";
      return attempt;
    }

    const Eigen::Vector3f center = initial_map_from_base.block<3, 1>(0, 3);
    const auto rough_submap = filter_z_band(
      crop_submap(rough_map, center, config_.submap_radius_xy, config_.submap_radius_z),
      config_.registration_z_min,
      config_.registration_z_max);
    const auto refine_submap = filter_z_band(
      crop_submap(refine_map, center, config_.submap_radius_xy, config_.submap_radius_z),
      config_.registration_z_min,
      config_.registration_z_max);
    attempt.submap_points = refine_submap->size();

    if (rough_submap->size() < static_cast<std::size_t>(config_.min_submap_points) ||
      refine_submap->size() < static_cast<std::size_t>(config_.min_submap_points))
    {
      attempt.decision = "skip_small_submap";
      return attempt;
    }

    // Scan is in base_link, submap in map; with base_link on the floor plane
    // and roll/pitch ~ 0 indoors the same z band applies to both.
    const auto banded_scan =
      filter_z_band(scan, config_.registration_z_min, config_.registration_z_max);
    const auto rough_scan = voxel_filter(banded_scan, config_.rough_scan_resolution);
    const auto refine_scan = voxel_filter(banded_scan, config_.refine_scan_resolution);
    if (rough_scan->size() < static_cast<std::size_t>(config_.min_scan_points) ||
      refine_scan->size() < static_cast<std::size_t>(config_.min_scan_points))
    {
      attempt.decision = "skip_small_filtered_scan";
      return attempt;
    }

    scan_registration::RegistrationOptions rough_options;
    rough_options.method = config_.registration_method;
    rough_options.resolution = config_.rough_map_resolution;
    rough_options.transformation_epsilon = config_.rough_transformation_epsilon;
    rough_options.rotation_epsilon = config_.rough_rotation_epsilon;
    rough_options.max_iterations = config_.rough_max_iteration;
    rough_options.max_correspondence_distance = config_.rough_max_correspondence_distance;
    rough_options.num_threads = config_.num_threads;
    rough_options.correspondence_randomness = config_.correspondence_randomness;

    auto rough_registration = scan_registration::create_registration(rough_options);
    rough_registration->setInputSource(rough_scan);
    rough_registration->setInputTarget(rough_submap);

    CloudType::Ptr rough_aligned(new CloudType);
    rough_registration->align(*rough_aligned, initial_map_from_base);
    if (!rough_registration->hasConverged()) {
      attempt.decision = "not_converged_rough";
      return attempt;
    }
    attempt.rough_rms = scan_registration::calc_fitness_score(
      rough_submap,
      rough_scan,
      matrix_to_isometry(rough_registration->getFinalTransformation()),
      config_.rough_max_correspondence_distance);
    if (attempt.rough_rms > config_.rough_score_thresh) {
      attempt.decision = "reject_rough_score";
      return attempt;
    }

    scan_registration::RegistrationOptions refine_options;
    refine_options.method = config_.registration_method;
    refine_options.resolution = config_.refine_map_resolution;
    refine_options.transformation_epsilon = config_.refine_transformation_epsilon;
    refine_options.rotation_epsilon = config_.refine_rotation_epsilon;
    refine_options.max_iterations = config_.refine_max_iteration;
    refine_options.max_correspondence_distance = config_.refine_max_correspondence_distance;
    refine_options.num_threads = config_.num_threads;
    refine_options.correspondence_randomness = config_.correspondence_randomness;

    auto refine_registration = scan_registration::create_registration(refine_options);
    refine_registration->setInputSource(refine_scan);
    refine_registration->setInputTarget(refine_submap);

    CloudType::Ptr refine_aligned(new CloudType);
    refine_registration->align(*refine_aligned, rough_registration->getFinalTransformation());
    if (!refine_registration->hasConverged()) {
      attempt.decision = "not_converged_refine";
      return attempt;
    }
    attempt.refine_rms = scan_registration::calc_fitness_score(
      refine_submap,
      refine_scan,
      matrix_to_isometry(refine_registration->getFinalTransformation()),
      config_.refine_max_correspondence_distance);
    // The refine score verdict belongs to the stateful ScoreGate, evaluated
    // under the state lock in align_timer_callback.

    aligned_map_from_base = refine_registration->getFinalTransformation();
    attempt.succeeded = true;
    return attempt;
  }

  static chassis_3d_localization::AlignmentGate::JumpMetrics compute_jump_metrics(
    const Matrix4f & predicted_map_from_base,
    const Matrix4f & aligned_map_from_base)
  {
    chassis_3d_localization::AlignmentGate::JumpMetrics metrics;
    metrics.translation =
      (aligned_map_from_base.block<3, 1>(0, 3) -
      predicted_map_from_base.block<3, 1>(0, 3)).norm();
    metrics.yaw_deg =
      std::fabs(
      normalize_angle(
        yaw_from_matrix(aligned_map_from_base) -
        yaw_from_matrix(predicted_map_from_base))) *
      180.0 / M_PI;
    return metrics;
  }

  bool load_map(const std::string & pcd_path, std::string & message)
  {
    if (!std::filesystem::exists(pcd_path)) {
      message = "pcd file not found: " + pcd_path;
      return false;
    }

    CloudType::Ptr loaded_map(new CloudType);
    if (pcl::io::loadPCDFile<PointType>(pcd_path, *loaded_map) != 0 ||
      loaded_map->empty())
    {
      message = "load pcd map failed: " + pcd_path;
      return false;
    }

    const auto rough_map = voxel_filter(loaded_map, config_.rough_map_resolution);
    const auto refine_map = voxel_filter(loaded_map, config_.refine_map_resolution);
    if (rough_map->empty() || refine_map->empty()) {
      message = "downsampled map is empty: " + pcd_path;
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      rough_map_ = rough_map;
      refine_map_ = refine_map;
      map_loaded_ = true;
      pending_initial_guess_available_ = false;
      localize_success_ = false;
      initialized_ = false;
      last_aligned_sequence_ = 0;
      ++state_epoch_;
      gate_.reset();
      score_gate_.reset();
    }

    publish_map_cloud(refine_map, time_to_msg(now()));
    message = "loaded pcd map";
    RCLCPP_INFO(
      get_logger(),
      "Loaded PCD map: raw=%zu rough=%zu refine=%zu path=%s",
      loaded_map->size(),
      rough_map->size(),
      refine_map->size(),
      pcd_path.c_str());
    return true;
  }

  void relocalize_callback(
    const std::shared_ptr<chassis_localization_msgs::srv::Relocalize::Request> request,
    std::shared_ptr<chassis_localization_msgs::srv::Relocalize::Response> response)
  {
    if (!std::filesystem::exists(request->pcd_path)) {
      response->success = false;
      response->message = "pcd file not found";
      return;
    }

    CloudType::Ptr loaded_map(new CloudType);
    if (pcl::io::loadPCDFile<PointType>(request->pcd_path, *loaded_map) != 0 ||
      loaded_map->empty())
    {
      response->success = false;
      response->message = "load pcd map failed";
      return;
    }

    const auto rough_map = voxel_filter(loaded_map, config_.rough_map_resolution);
    const auto refine_map = voxel_filter(loaded_map, config_.refine_map_resolution);
    if (rough_map->empty() || refine_map->empty()) {
      response->success = false;
      response->message = "downsampled map is empty";
      return;
    }

    const Matrix4f initial_guess = make_pose(
      request->x,
      request->y,
      request->z,
      request->yaw,
      request->pitch,
      request->roll);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      rough_map_ = rough_map;
      refine_map_ = refine_map;
      map_loaded_ = true;
      pending_initial_guess_ = initial_guess;
      pending_initial_guess_available_ = true;
      localize_success_ = false;
      initialized_ = false;
      last_aligned_sequence_ = 0;
      ++state_epoch_;
      gate_.reset();
      score_gate_.reset();
    }

    response->success = true;
    response->message = "relocalize request accepted";
    publish_map_cloud(refine_map, time_to_msg(now()));
    RCLCPP_INFO(
      get_logger(),
      "Loaded PCD map: raw=%zu rough=%zu refine=%zu path=%s",
      loaded_map->size(),
      rough_map->size(),
      refine_map->size(),
      request->pcd_path.c_str());
  }

  void relocalize_check_callback(
    const std::shared_ptr<chassis_localization_msgs::srv::IsValid::Request> request,
    std::shared_ptr<chassis_localization_msgs::srv::IsValid::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response->valid = request->code == 1 || localize_success_;
  }

  void publish_tf(const Matrix4f & map_from_odom)
  {
    // Future-dated so lookups between alignment updates stay valid and
    // repeated publishes never share a stamp (avoids TF_REPEATED_DATA).
    const auto stamp = time_to_msg(
      now() + rclcpp::Duration::from_seconds(config_.transform_tolerance));
    tf_broadcaster_->sendTransform(
      matrix_to_transform(map_from_odom, stamp, config_.map_frame, config_.odom_frame));
  }

  void publish_localized_result(
    const builtin_interfaces::msg::Time & stamp,
    const Matrix4f & map_from_base)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = config_.map_frame;
    odom.child_frame_id = config_.base_frame;
    set_odom_pose(odom, map_from_base);
    localized_pub_->publish(odom);
  }

  void publish_map_cloud(
    const CloudType::ConstPtr & map,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (!map || map->empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.map_frame;
    map_cloud_pub_->publish(msg);
    RCLCPP_INFO(
      get_logger(),
      "Published latched map_cloud once: points=%zu",
      map->size());
  }

  LocalizerConfig config_;
  std::mutex mutex_;

  bool has_frame_ = false;
  bool map_loaded_ = false;
  bool initialized_ = false;
  bool localize_success_ = false;
  bool pending_initial_guess_available_ = false;

  FrameData latest_frame_;
  Matrix4f pending_initial_guess_ = Matrix4f::Identity();
  Matrix4f map_from_odom_ = Matrix4f::Identity();
  CloudType::Ptr rough_map_;
  CloudType::Ptr refine_map_;

  std::uint64_t last_aligned_sequence_ = 0;
  // Bumped by initialpose/relocalize/map loads; alignment results computed
  // against an older epoch are discarded.
  std::uint64_t state_epoch_ = 0;
  chassis_3d_localization::AlignmentGate gate_;
  chassis_3d_localization::ScoreGate score_gate_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localized_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Service<chassis_localization_msgs::srv::Relocalize>::SharedPtr relocalize_srv_;
  rclcpp::Service<chassis_localization_msgs::srv::IsValid>::SharedPtr relocalize_check_srv_;
  rclcpp::CallbackGroup::SharedPtr align_callback_group_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr align_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Chassis3DLocalizer>();
  // Two threads: one keeps TF/subscriptions responsive while the other runs
  // the VGICP alignment callback group.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
