#include "agv2_pkg/steering_channel.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace agv2_pkg {
SteeringChannel::SteeringChannel(SteeringConfig c) : cfg_(c) {
  if (c.response_ms <= 0 || c.cycle_ms < c.response_ms || c.stale_ms < c.cycle_ms ||
      c.status_ms < 1000 || c.backoff_ms < c.response_ms ||
      c.max_backoff_ms < c.backoff_ms || c.recovery_ms <= c.max_backoff_ms ||
      c.verify_replies < 1) throw std::invalid_argument("Invalid steering recovery parameters");
}
bool SteeringChannel::fresh(int64_t t, int64_t now, int64_t limit) const {
  return t >= 0 && now >= t && now - t <= limit;
}
bool SteeringChannel::ready(int64_t now) const {
  return enabled_ && !exhausted_ && !fault_ && recovery_start_ < 0 &&
         confirmations_ >= cfg_.verify_replies &&
         fresh(last_position_, now, cfg_.stale_ms) &&
         fresh(last_ack_, now, cfg_.stale_ms) &&
         fresh(last_status_, now, cfg_.status_ms + cfg_.stale_ms);
}
void SteeringChannel::recover(int64_t now) {
  if (recovery_start_ >= 0 || exhausted_) return;
  recovery_start_ = now;
  enabled_ = false;
  confirmations_ = 0;
  last_ack_ = -1;
  last_status_ = -1;
  last_status_query_ = -1;
  target_ = measured_;
}
void SteeringChannel::retry(int64_t now) {
  // Used only by the existing explicit enable edge. Does not clear bus faults.
  if (fresh(last_status_, now, cfg_.status_ms + cfg_.stale_ms) && !last_error_) fault_ = false;
  if (!exhausted_) return;
  exhausted_ = false;
  recovery_start_ = now;
  next_send_ = now;
  backoff_ = 0;
}
std::optional<SteeringRequest> SteeringChannel::send(uint8_t cmd, int64_t now, double angle) {
  pending_ = cmd;
  sent_ = now;
  if (cmd == 0x94) last_query_ = now;
  if (cmd == 0x9a) last_status_query_ = now;
  if (cmd == 0xa4) last_target_ = now;
  return SteeringRequest{cmd, angle};
}
std::optional<SteeringRequest> SteeringChannel::poll(int64_t now, bool permit) {
  if (!permit) recovery_start_ = -1;  // no recovery deadline while operator is idle/locked
  if (permit && ever_ready_ && !exhausted_ && recovery_start_ < 0 &&
      (!fresh(last_position_, now, cfg_.stale_ms) ||
       !fresh(last_ack_, now, cfg_.stale_ms) ||
       !fresh(last_status_, now, cfg_.status_ms + cfg_.stale_ms))) recover(now);
  if (recovery_start_ >= 0 && now - recovery_start_ >= cfg_.recovery_ms) exhausted_ = true;
  if (pending_) {
    if (now - sent_ < cfg_.response_ms) return {};
    const auto expired = pending_;
    pending_ = 0;
    // Legacy 0x88 has no guaranteed ACK. Query position/status to verify instead.
    if (expired != 0x88) {
      ++timeouts_;
      backoff_ = backoff_ ? std::min(cfg_.max_backoff_ms, backoff_ * 2) : cfg_.backoff_ms;
      next_send_ = now + backoff_;
    }
  }
  if (exhausted_ || now < next_send_) return {};
  // Position is always acquired first. Diagnostic queries share this same owner.
  if (last_query_ < 0 || now - last_query_ >= cfg_.cycle_ms) return send(0x94, now);
  if (last_status_query_ < 0 || now - last_status_query_ >= cfg_.status_ms) return send(0x9a, now);
  if (!permit || fault_ || !fresh(last_position_, now, cfg_.stale_ms) ||
      !fresh(last_status_, now, cfg_.status_ms + cfg_.stale_ms)) return {};
  if (!enabled_) {
    enabled_ = true;
    if (recovery_start_ < 0) recovery_start_ = now;
    confirmations_ = 0;
    last_ack_ = -1;
    last_target_ = -1;
    return send(0x88, now);
  }
  if (confirmations_ < cfg_.verify_replies) return {};
  if (last_target_ < 0 || now - last_target_ >= cfg_.cycle_ms) {
    // First position command after enabling/recovering holds measured position.
    return send(0xa4, now, last_ack_ < 0 ? measured_ : target_);
  }
  return {};
}
bool SteeringChannel::receive(uint8_t cmd, int64_t now, double angle, uint8_t error) {
  // Never treat an unrelated/late packet as a fresh answer to a new transaction.
  if (!pending_ || cmd != pending_ || now < sent_ || now - sent_ >= cfg_.response_ms) return false;
  if (cmd == 0x94 && !std::isfinite(angle)) return false;
  pending_ = 0;
  next_send_ = now + 2;  // callback/timer scheduling, never a blocking sleep
  backoff_ = 0;
  if (cmd == 0x94) {
    last_position_ = now;
    measured_ = angle;
    if (enabled_) confirmations_ = std::min(cfg_.verify_replies, confirmations_ + 1);
  } else if (cmd == 0x9a) {
    last_status_ = now;
    last_error_ = error;
    // Preserve raw legacy error byte; do not infer temperature or clear faults.
    if (error) fault_ = true;
  } else if (cmd == 0xa4 && enabled_ && confirmations_ >= cfg_.verify_replies) {
    last_ack_ = now;
    if (!fault_) {
      recovery_start_ = -1;
      ever_ready_ = true;
    }
  }
  return true;
}
const char* SteeringChannel::phase(int64_t now) const {
  if (fault_) return "hardware_fault_latched";
  if (exhausted_) return "recovery_exhausted_operator_retry_required";
  if (recovery_start_ >= 0) return ever_ready_ ? "recovering" : "verifying";
  if (ready(now)) return "ready";
  return enabled_ ? "verifying" : "preflight";
}
}  // namespace agv2_pkg
