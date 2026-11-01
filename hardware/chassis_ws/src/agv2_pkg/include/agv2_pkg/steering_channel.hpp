#pragma once
#include <cstdint>
#include <optional>

namespace agv2_pkg {
struct SteeringConfig {
  int64_t response_ms{20};
  int64_t cycle_ms{50};
  int64_t status_ms{1000};
  int64_t stale_ms{300};
  int64_t backoff_ms{100};
  int64_t max_backoff_ms{1000};
  int64_t recovery_ms{10000};
  int verify_replies{2};
};
struct SteeringRequest { uint8_t command; double angle_rad; };

// One in-flight transaction per motor. No FIFO of old movement targets.
// Call only with true bus replies (SocketCAN sender loopback MUST be disabled).
class SteeringChannel {
 public:
  explicit SteeringChannel(SteeringConfig config = {});
  void target(double radians) { target_ = radians; }
  std::optional<SteeringRequest> poll(int64_t now, bool permit_actuation);
  bool receive(uint8_t command, int64_t now, double angle_rad = 0,
               uint8_t error_raw = 0);
  bool ready(int64_t now) const;
  void retry(int64_t now);  // explicit operator enable; never clears a live fault
  const char* phase(int64_t now) const;
  bool in_flight() const { return pending_ != 0; }
  uint8_t pending_command() const { return pending_; }
  uint64_t timeouts() const { return timeouts_; }
  double measured() const { return measured_; }
 private:
  bool fresh(int64_t time, int64_t now, int64_t limit) const;
  void recover(int64_t now);
  std::optional<SteeringRequest> send(uint8_t command, int64_t now, double angle = 0);
  SteeringConfig cfg_;
  uint8_t pending_{0};
  int64_t sent_{0}, next_send_{0}, last_query_{-1}, last_status_query_{-1};
  int64_t last_position_{-1}, last_status_{-1}, last_target_{-1}, last_ack_{-1};
  int64_t recovery_start_{-1}, backoff_{0};
  bool enabled_{false}, ever_ready_{false}, exhausted_{false}, fault_{false};
  uint8_t last_error_{0};
  int confirmations_{0};
  uint64_t timeouts_{0};
  double target_{0}, measured_{0};
};
}  // namespace agv2_pkg
