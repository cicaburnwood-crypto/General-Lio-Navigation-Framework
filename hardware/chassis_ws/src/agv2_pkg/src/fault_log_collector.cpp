// Standalone observer; does not subscribe to to_can_bus or publish control messages.
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <agv2_pkg/msg/chassis_telemetry.hpp>
#include <agv2_pkg/msg/wheel_command.hpp>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/prctl.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
using J = nlohmann::json;
using namespace std::chrono_literals;
template<class T> std::string hex_gid(const T &a) {
  std::ostringstream s; s << std::hex << std::setfill('0');
  for (auto b : a) s << std::setw(2) << static_cast<unsigned>(b);
  return s.str();
}
static bool relevant(const std::string &n) {
  return n.find("agv2_control") != std::string::npos ||
    n.find("socket_can") != std::string::npos || n == "joy_node" || n == "/joy_node";
}
class Collector : public rclcpp::Node {
 public:
  explicit Collector(const std::string &root) : Node("chassis_fault_collector"), root_(root) {
    writer_ = std::thread([this] {
      for (;;) {
        std::unique_lock<std::mutex> l(mu_);
        cv_.wait(l, [this] { return quit_ || !queue_.empty(); });
        if (quit_ && queue_.empty()) return;
        auto line = std::move(queue_.front()); queue_.pop_front(); l.unlock();
        std::cout << line << std::endl;
      }
    });
    sub<geometry_msgs::msg::Twist>("/cmd_vel", [](const auto &m) {
      return J{{"linear", {m.linear.x,m.linear.y,m.linear.z}}, {"angular",{m.angular.x,m.angular.y,m.angular.z}}};
    });
    sub<sensor_msgs::msg::Joy>("/joy", [](const auto &m) { return J{{"axes",m.axes},{"buttons",m.buttons}}; });
    sub<agv2_pkg::msg::WheelCommand>("/wheel_command", [](const auto &m) {
      return J{{"front_steer_angle",m.front_steer_angle},{"rear_steer_angle",m.rear_steer_angle},
               {"front_wheel_speed",m.front_wheel_speed},{"rear_wheel_speed",m.rear_wheel_speed}};
    });
    sub<agv2_pkg::msg::ChassisTelemetry>("/agv2/chassis_telemetry", [](const auto &m) {
      J j;
        j["mode"] = m.mode;
        j["fsm_state"] = m.fsm_state;
        j["active_source"] = m.active_source;
        j["have_input"] = m.have_input;
        j["steer_front_can_id"] = m.steer_front_can_id;
        j["steer_rear_can_id"] = m.steer_rear_can_id;
        j["wheel_front_command_can_id"] = m.wheel_front_command_can_id;
        j["wheel_rear_command_can_id"] = m.wheel_rear_command_can_id;
        j["wheel_front_status_can_id"] = m.wheel_front_status_can_id;
        j["wheel_rear_status_can_id"] = m.wheel_rear_status_can_id;
        j["requested_front_steer_rad"] = m.requested_front_steer_rad;
        j["requested_rear_steer_rad"] = m.requested_rear_steer_rad;
        j["requested_front_speed_mps"] = m.requested_front_speed_mps;
        j["requested_rear_speed_mps"] = m.requested_rear_speed_mps;
        j["commanded_front_steer_rad"] = m.commanded_front_steer_rad;
        j["commanded_rear_steer_rad"] = m.commanded_rear_steer_rad;
        j["commanded_front_speed_mps"] = m.commanded_front_speed_mps;
        j["commanded_rear_speed_mps"] = m.commanded_rear_speed_mps;
        j["front_wheel_feedback_seen"] = m.front_wheel_feedback_seen;
        j["rear_wheel_feedback_seen"] = m.rear_wheel_feedback_seen;
        j["front_wheel_feedback_count"] = m.front_wheel_feedback_count;
        j["rear_wheel_feedback_count"] = m.rear_wheel_feedback_count;
        j["front_wheel_feedback_age_ms"] = m.front_wheel_feedback_age_ms;
        j["rear_wheel_feedback_age_ms"] = m.rear_wheel_feedback_age_ms;
        j["front_wheel_status_word"] = m.front_wheel_status_word;
        j["rear_wheel_status_word"] = m.rear_wheel_status_word;
        j["front_wheel_error_code"] = m.front_wheel_error_code;
        j["rear_wheel_error_code"] = m.rear_wheel_error_code;
        j["front_wheel_actual_velocity_raw"] = m.front_wheel_actual_velocity_raw;
        j["rear_wheel_actual_velocity_raw"] = m.rear_wheel_actual_velocity_raw;
        j["front_steer_feedback_seen"] = m.front_steer_feedback_seen;
        j["rear_steer_feedback_seen"] = m.rear_steer_feedback_seen;
        j["front_steer_feedback_count"] = m.front_steer_feedback_count;
        j["rear_steer_feedback_count"] = m.rear_steer_feedback_count;
        j["front_steer_loopback_count"] = m.front_steer_loopback_count;
        j["rear_steer_loopback_count"] = m.rear_steer_loopback_count;
        j["front_steer_feedback_age_ms"] = m.front_steer_feedback_age_ms;
        j["rear_steer_feedback_age_ms"] = m.rear_steer_feedback_age_ms;
        j["front_steer_feedback_deg"] = m.front_steer_feedback_deg;
        j["rear_steer_feedback_deg"] = m.rear_steer_feedback_deg;
        j["front_steer_error_deg"] = m.front_steer_error_deg;
        j["rear_steer_error_deg"] = m.rear_steer_error_deg;
        j["can_error_frame_count"] = m.can_error_frame_count;
        j["diagnostic_flags"] = m.diagnostic_flags;
      j["header"] = {{"sec",m.header.stamp.sec},{"nanosec",m.header.stamp.nanosec},{"frame_id",m.header.frame_id}};
      return j;
    });
    sub<std_msgs::msg::Float64>("/Battery_SOC_STATE", [](const auto &m) { return J{{"data",m.data}}; });
    sub<std_msgs::msg::String>("/agv2/log_event", [](const auto &m) { return J{{"data",m.data}}; });
    sub<rcl_interfaces::msg::Log>("/rosout", [](const auto &m) {
      if (!relevant(m.name)) return J();
      return J{{"name",m.name},{"level",m.level},{"message",m.msg},{"file",m.file},{"function",m.function},{"line",m.line}};
    });
    sub<rcl_interfaces::msg::ParameterEvent>("/parameter_events", [](const auto &m) {
      if (!relevant(m.node)) return J();
      J changes=J::array();
      for (const auto *group : {&m.new_parameters,&m.changed_parameters,&m.deleted_parameters})
        for (const auto &p : *group) changes.push_back({{"name",p.name},{"type",p.value.type},
          {"integer",p.value.integer_value},{"double",p.value.double_value},{"bool",p.value.bool_value},
          {"string",p.value.string_value}});
      return J{{"node",m.node},{"changes",changes}};
    });
    timer_ = create_wall_timer(500ms, [this] {
      J graph=J::object();
      for (auto topic : {"/joy","/cmd_vel","/wheel_command","/to_can_bus","/agv2/chassis_telemetry","/from_can_bus"}) {
        graph[topic] = J::array();
        for (const auto &e : get_publishers_info_by_topic(topic)) {
          const auto &q=e.qos_profile().get_rmw_qos_profile();
          graph[topic].push_back({{"node",e.node_name()},{"namespace",e.node_namespace()},
            {"gid",hex_gid(e.endpoint_gid())},{"topic_type",e.topic_type()},{"host","unknown"},
            {"qos",{{"reliability",static_cast<int>(q.reliability)},{"durability",static_cast<int>(q.durability)},
                    {"history",static_cast<int>(q.history)},{"depth",q.depth}}}});
        }
      }
      J health; { std::lock_guard<std::mutex> l(mu_); health={{"queued",queue_.size()},{"dropped",dropped_}}; }
      emit("events","collector_health",health);
      emit("events","ros_graph",graph);
    });
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_ = create_service<std_srvs::srv::Trigger>("/agv2/save_fault_log",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        int fd=socket(AF_UNIX,SOCK_STREAM | SOCK_CLOEXEC,0);
        sockaddr_un addr{}; addr.sun_family=AF_UNIX;
        auto path=root_+"/.control";
        if (path.size()>=sizeof(addr.sun_path)) { response->message="IPC path too long"; close(fd); return; }
        std::copy(path.begin(),path.end(),addr.sun_path);
        timeval timeout{8,0}; setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
        if (connect(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0) {
          response->message="Recorder unavailable; nothing claimed saved"; close(fd); return;
        }
        const std::string req="{\"command\":\"save\",\"reason\":\"manual_ros_service\"}\n";
        if (send(fd,req.data(),req.size(),MSG_NOSIGNAL)!=static_cast<ssize_t>(req.size())) {
          response->message="Recorder request failed"; close(fd); return;
        }
        std::string result; char buf[4096]; ssize_t n;
        while ((n=recv(fd,buf,sizeof(buf),0))>0 && result.size()<65536) {
          result.append(buf,n); if (result.find('\n')!=std::string::npos) break;
        }
        close(fd);
        try { auto j=J::parse(result); response->success=j.value("ok",false); response->message=result; }
        catch (...) { response->message="Persistence acknowledgment timed out; package may still be pending"; }
      }, rmw_qos_profile_services_default, service_group_);
  }
  ~Collector() override {
    { std::lock_guard<std::mutex> l(mu_); quit_=true; }
    cv_.notify_all(); if (writer_.joinable()) writer_.join();
  }
 private:
  void emit(const std::string &kind, const std::string &source, J payload) {
    auto now=std::chrono::steady_clock::now().time_since_epoch();
    J j={{"kind",kind},{"source",source},{"payload",payload},
         {"capture_monotonic_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()}};
    std::lock_guard<std::mutex> l(mu_);
    if (queue_.size()>=8000) { queue_.pop_front(); ++dropped_; }
    queue_.push_back(j.dump(-1,' ',false,J::error_handler_t::replace)); cv_.notify_one();
  }
  template<class T,class F> void sub(const std::string &topic,F convert) {
    subs_.push_back(create_subscription<T>(topic,rclcpp::SensorDataQoS().keep_last(500),
      [this,topic,convert](std::shared_ptr<T> m,const rclcpp::MessageInfo &info) {
        auto payload=convert(*m); if (payload.is_null()) return;
        const auto &i=info.get_rmw_message_info();
        payload["_publisher_gid"]=hex_gid(i.publisher_gid.data);
        payload["_source_timestamp_ns"]=i.source_timestamp;
        payload["_received_timestamp_ns"]=i.received_timestamp;
        emit("state",topic,payload);
      }));
  }
  std::string root_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  std::mutex mu_; std::condition_variable cv_; std::deque<std::string> queue_;
  std::thread writer_; bool quit_=false; uint64_t dropped_=0;
};
int main(int argc,char **argv) {
  if (argc<2) return 2;
  const auto parent = getppid();
  if (prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 || getppid() != parent || parent == 1) return 3;
  std::string root=argv[1];
  rclcpp::init(argc,argv);
  auto node=std::make_shared<Collector>(root);
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(),2);
  ex.add_node(node); ex.spin(); rclcpp::shutdown();
}
