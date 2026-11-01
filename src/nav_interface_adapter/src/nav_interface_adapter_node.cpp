#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{

constexpr char kPointCloud2Mode[] = "pointcloud2";
constexpr char kLivoxCustomMode[] = "livox_custom";

bool is_finite_point(const float x, const float y, const float z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
  return it == cloud.fields.end() ? nullptr : &(*it);
}

float read_numeric_field(
  const uint8_t * point,
  const sensor_msgs::msg::PointField & field,
  const bool bigendian)
{
  (void)bigendian;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32: {
      float value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return value;
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      double value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<float>(*(point + field.offset));
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<float>(*reinterpret_cast<const int8_t *>(point + field.offset));
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::msg::PointField::INT16: {
      int16_t value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::msg::PointField::UINT32: {
      uint32_t value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::msg::PointField::INT32: {
      int32_t value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return static_cast<float>(value);
    }
    default:
      return 0.0F;
  }
}

tf2::Transform make_transform(
  const std::vector<double> & xyz,
  const std::vector<double> & rpy)
{
  tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  q.normalize();
  return tf2::Transform(q, tf2::Vector3(xyz[0], xyz[1], xyz[2]));
}

tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose)
{
  return tf2::Transform(
    tf2::Quaternion(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::msg::TransformStamped to_transform_msg(
  const tf2::Transform & tf,
  const rclcpp::Time & stamp,
  const std::string & parent,
  const std::string & child)
{
  geometry_msgs::msg::TransformStamped msg;
  const auto origin = tf.getOrigin();
  const auto rotation = tf.getRotation();
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = origin.x();
  msg.transform.translation.y = origin.y();
  msg.transform.translation.z = origin.z();
  msg.transform.rotation.x = rotation.x();
  msg.transform.rotation.y = rotation.y();
  msg.transform.rotation.z = rotation.z();
  msg.transform.rotation.w = rotation.w();
  return msg;
}

void set_pose(geometry_msgs::msg::Pose & pose, const tf2::Transform & tf)
{
  const auto origin = tf.getOrigin();
  const auto rotation = tf.getRotation();
  pose.position.x = origin.x();
  pose.position.y = origin.y();
  pose.position.z = origin.z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
}

}  // namespace

class NavInterfaceAdapter : public rclcpp::Node
{
public:
  NavInterfaceAdapter()
  : Node("nav_interface_adapter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/Odometry");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/odom");
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/livox/lidar");
    output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/sensors/cloud");
    output_scan_topic_ = declare_parameter<std::string>("output_scan_topic", "/scan");
    cloud_input_type_ = declare_parameter<std::string>("cloud_input_type", kLivoxCustomMode);
    output_cloud_frame_ = declare_parameter<std::string>("output_cloud_frame", "");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "livox_frame");
    scan_target_frame_ = declare_parameter<std::string>("scan_target_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_lidar_tf_ = declare_parameter<bool>("publish_lidar_tf", true);
    base_to_lidar_xyz_ =
      declare_parameter<std::vector<double>>("base_to_lidar_xyz", {0.0, 0.0, 0.0});
    base_to_lidar_rpy_ =
      declare_parameter<std::vector<double>>("base_to_lidar_rpy", {0.0, 0.0, 0.0});
    min_height_ = declare_parameter<double>("min_height", 0.3);
    max_height_ = declare_parameter<double>("max_height", 0.8);
    angle_min_ = declare_parameter<double>("angle_min", -3.14159);
    angle_max_ = declare_parameter<double>("angle_max", 3.14159);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.0087);
    scan_time_ = declare_parameter<double>("scan_time", 0.1);
    range_min_ = declare_parameter<double>("range_min", 0.3);
    range_max_ = declare_parameter<double>("range_max", 40.0);
    use_inf_ = declare_parameter<bool>("use_inf", true);
    inf_epsilon_ = declare_parameter<double>("inf_epsilon", 1.0);
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.05);

    validate_parameters();
    base_to_lidar_tf_ = make_transform(base_to_lidar_xyz_, base_to_lidar_rpy_);
    lidar_to_base_tf_ = base_to_lidar_tf_.inverse();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, rclcpp::SensorDataQoS());
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_scan_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publish_lidar_static_tf();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 20,
      std::bind(&NavInterfaceAdapter::odom_callback, this, std::placeholders::_1));

    if (cloud_input_type_ == kPointCloud2Mode) {
      pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&NavInterfaceAdapter::pointcloud2_callback, this, std::placeholders::_1));
    } else if (cloud_input_type_ == kLivoxCustomMode) {
      livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&NavInterfaceAdapter::livox_callback, this, std::placeholders::_1));
    } else {
      throw std::invalid_argument(
        "cloud_input_type must be 'pointcloud2' or 'livox_custom'");
    }

    RCLCPP_INFO(
      get_logger(),
      "Adapter ready: odom %s -> %s, cloud %s (%s) -> %s[%s] + %s",
      input_odom_topic_.c_str(), output_odom_topic_.c_str(),
      input_cloud_topic_.c_str(), cloud_input_type_.c_str(),
      output_cloud_topic_.c_str(),
      output_cloud_frame_.empty() ? "source frame" : output_cloud_frame_.c_str(),
      output_scan_topic_.c_str());
  }

private:
  void validate_parameters() const
  {
    if (angle_increment_ <= 0.0) {
      throw std::invalid_argument("angle_increment must be positive");
    }
    if (angle_max_ <= angle_min_) {
      throw std::invalid_argument("angle_max must be greater than angle_min");
    }
    if (range_max_ <= range_min_) {
      throw std::invalid_argument("range_max must be greater than range_min");
    }
    if (max_height_ < min_height_) {
      throw std::invalid_argument("max_height must be greater than or equal to min_height");
    }
    if (base_to_lidar_xyz_.size() != 3 || base_to_lidar_rpy_.size() != 3) {
      throw std::invalid_argument("base_to_lidar_xyz and base_to_lidar_rpy must have 3 values");
    }
  }

  void publish_lidar_static_tf()
  {
    if (publish_lidar_tf_ && !cloud_frame_.empty() && cloud_frame_ != base_frame_) {
      static_tf_broadcaster_->sendTransform(
        to_transform_msg(base_to_lidar_tf_, now(), base_frame_, cloud_frame_));
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto odom = *msg;
    const auto odom_to_base = pose_to_transform(msg->pose.pose) * lidar_to_base_tf_;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    set_pose(odom.pose.pose, odom_to_base);
    odom_pub_->publish(odom);

    if (!publish_tf_) {
      return;
    }

    tf_broadcaster_->sendTransform(
      to_transform_msg(odom_to_base, odom.header.stamp, odom_frame_, base_frame_));
  }

  void pointcloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto cloud = *msg;
    if (!cloud_frame_.empty()) {
      cloud.header.frame_id = cloud_frame_;
    }
    publish_cloud_and_scan(cloud);
  }

  void livox_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    if (!cloud_frame_.empty()) {
      cloud.header.frame_id = cloud_frame_;
    }
    cloud.height = 1;
    cloud.width = msg->point_num;
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(msg->point_num);

    sensor_msgs::PointCloud2Iterator<float> x_iter(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iter(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iter(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity_iter(cloud, "intensity");

    const auto count = std::min<std::size_t>(msg->point_num, msg->points.size());
    for (std::size_t i = 0; i < count; ++i, ++x_iter, ++y_iter, ++z_iter, ++intensity_iter) {
      const auto & point = msg->points[i];
      *x_iter = point.x;
      *y_iter = point.y;
      *z_iter = point.z;
      *intensity_iter = static_cast<float>(point.reflectivity);
    }

    if (count != msg->point_num) {
      modifier.resize(count);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Livox point_num (%u) does not match points size (%zu); publishing %zu points",
        msg->point_num, msg->points.size(), count);
    }

    publish_cloud_and_scan(cloud);
  }

  void publish_cloud_and_scan(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    sensor_msgs::msg::PointCloud2 output_cloud;
    if (!transform_cloud_to_frame(
        cloud, output_cloud_frame_, output_cloud, "output cloud publication"))
    {
      return;
    }

    cloud_pub_->publish(output_cloud);
    auto scan_cloud = output_cloud;

    if (!transform_cloud_to_frame(
        scan_cloud, scan_target_frame_, scan_cloud, "scan projection"))
    {
      return;
    }

    scan_pub_->publish(project_cloud_to_scan(scan_cloud));
  }

  bool transform_cloud_to_frame(
    const sensor_msgs::msg::PointCloud2 & input,
    const std::string & target_frame,
    sensor_msgs::msg::PointCloud2 & output,
    const char * purpose)
  {
    const auto source = input;
    output = source;
    if (target_frame.empty() || source.header.frame_id == target_frame) {
      return true;
    }

    try {
      const auto timeout =
        rclcpp::Duration::from_seconds(std::max(0.0, transform_tolerance_));
      const auto transform = tf_buffer_.lookupTransform(
        target_frame, source.header.frame_id, source.header.stamp, timeout);
      tf2::doTransform(source, output, transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping %s: cannot transform cloud from '%s' to '%s': %s",
        purpose, source.header.frame_id.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  sensor_msgs::msg::LaserScan project_cloud_to_scan(
    const sensor_msgs::msg::PointCloud2 & cloud)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header = cloud.header;
    if (!scan_target_frame_.empty()) {
      scan.header.frame_id = scan_target_frame_;
    }
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_max_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = 0.0F;
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);

    const auto bin_count = static_cast<std::size_t>(
      std::ceil((angle_max_ - angle_min_) / angle_increment_));
    const auto default_range = use_inf_
      ? std::numeric_limits<float>::infinity()
      : static_cast<float>(range_max_ + inf_epsilon_);
    scan.ranges.assign(bin_count, default_range);
    scan.intensities.assign(bin_count, 0.0F);

    const auto * x_field = find_field(cloud, "x");
    const auto * y_field = find_field(cloud, "y");
    const auto * z_field = find_field(cloud, "z");
    const auto * intensity_field = find_field(cloud, "intensity");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 is missing x/y/z fields; publishing an empty scan");
      return scan;
    }

    const auto point_count = static_cast<std::size_t>(cloud.width) *
      static_cast<std::size_t>(cloud.height);
    for (std::size_t i = 0; i < point_count; ++i) {
      const auto * point = cloud.data.data() + (i * cloud.point_step);
      const auto x = read_numeric_field(point, *x_field, cloud.is_bigendian);
      const auto y = read_numeric_field(point, *y_field, cloud.is_bigendian);
      const auto z = read_numeric_field(point, *z_field, cloud.is_bigendian);

      if (!is_finite_point(x, y, z) || z < min_height_ || z > max_height_) {
        continue;
      }

      const auto range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_) {
        continue;
      }

      const auto angle = std::atan2(y, x);
      if (angle < angle_min_ || angle > angle_max_) {
        continue;
      }

      const auto index = static_cast<std::size_t>(
        std::floor((angle - angle_min_) / angle_increment_));
      if (index >= scan.ranges.size() || range >= scan.ranges[index]) {
        continue;
      }

      scan.ranges[index] = static_cast<float>(range);
      if (intensity_field != nullptr) {
        scan.intensities[index] = read_numeric_field(
          point, *intensity_field, cloud.is_bigendian);
      }
    }

    return scan;
  }

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string output_scan_topic_;
  std::string cloud_input_type_;
  std::string output_cloud_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string cloud_frame_;
  std::string scan_target_frame_;
  bool publish_tf_{true};
  bool publish_lidar_tf_{true};
  std::vector<double> base_to_lidar_xyz_{0.0, 0.0, 0.0};
  std::vector<double> base_to_lidar_rpy_{0.0, 0.0, 0.0};
  double min_height_{0.3};
  double max_height_{0.8};
  double angle_min_{-3.14159};
  double angle_max_{3.14159};
  double angle_increment_{0.0087};
  double scan_time_{0.1};
  double range_min_{0.3};
  double range_max_{40.0};
  bool use_inf_{true};
  double inf_epsilon_{1.0};
  double transform_tolerance_{0.05};
  tf2::Transform base_to_lidar_tf_;
  tf2::Transform lidar_to_base_tf_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavInterfaceAdapter>());
  rclcpp::shutdown();
  return 0;
}
