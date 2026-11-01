#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_self_filter/self_filter.hpp"
#include "rclcpp/rclcpp.hpp"

class LivoxSelfFilterNode : public rclcpp::Node
{
public:
  LivoxSelfFilterNode()
  : Node("livox_self_filter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar_filtered");
    const auto base_to_lidar_xyz =
      declare_parameter<std::vector<double>>("base_to_lidar_xyz", {0.0, 0.0, 0.0});
    const auto base_to_lidar_rpy =
      declare_parameter<std::vector<double>>("base_to_lidar_rpy", {0.0, 0.0, 0.0});
    const auto boxes_base_link =
      declare_parameter<std::vector<double>>("boxes_base_link", std::vector<double>{});

    base_from_lidar_ = livox_self_filter::make_transform(
      base_to_lidar_xyz, base_to_lidar_rpy);
    boxes_ = livox_self_filter::boxes_from_flat(boxes_base_link);

    const auto qos = rclcpp::QoS(20);
    pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic_, qos);
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, qos,
      std::bind(&LivoxSelfFilterNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Livox self filter ready: %s -> %s, boxes=%zu",
      input_topic_.c_str(),
      output_topic_.c_str(),
      boxes_.size());
  }

private:
  void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    const auto output = livox_self_filter::filter_message(
      *msg, base_from_lidar_, boxes_);
    pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  tf2::Transform base_from_lidar_;
  std::vector<livox_self_filter::Box> boxes_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LivoxSelfFilterNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("livox_self_filter"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
