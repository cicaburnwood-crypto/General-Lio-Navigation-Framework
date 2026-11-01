#ifndef CHASSIS_3D_LOCALIZATION__ALIGNMENT_GATE_HPP_
#define CHASSIS_3D_LOCALIZATION__ALIGNMENT_GATE_HPP_

namespace chassis_3d_localization
{

// Jump gate with stuck-state recovery for scan-to-map alignment results.
//
// A plain jump gate deadlocks: once odometry drift exceeds the jump limits,
// every subsequent correction is rejected and the drift only grows. This gate
// therefore counts consecutive jump rejections and converts the Nth one into
// an override so localization can re-attach to the map. The caller must only
// call evaluate() for alignments whose fitness scores already passed, so an
// override never admits a low-quality result.
class AlignmentGate
{
public:
  struct Config
  {
    double max_translation_jump = 0.5;  // meters
    double max_yaw_jump_deg = 10.0;     // degrees
    // Consecutive jump rejections converted into an override. <= 0 disables
    // the override (legacy behavior: reject forever).
    int max_consecutive_rejections = 5;
  };

  struct JumpMetrics
  {
    double translation = 0.0;  // meters between predicted and aligned pose
    double yaw_deg = 0.0;      // absolute yaw difference, degrees
  };

  enum class Decision
  {
    kAccepted,
    kRejectedJump,
    kJumpOverride,
  };

  AlignmentGate() = default;

  explicit AlignmentGate(const Config & config)
  : config_(config)
  {
  }

  // bypass_jump_check is set when the alignment started from an external
  // initial guess (/initialpose or relocalize) instead of the tracked pose.
  Decision evaluate(const JumpMetrics & metrics, const bool bypass_jump_check)
  {
    if (bypass_jump_check || within_limits(metrics)) {
      consecutive_rejections_ = 0;
      return Decision::kAccepted;
    }

    ++consecutive_rejections_;
    if (config_.max_consecutive_rejections > 0 &&
      consecutive_rejections_ >= config_.max_consecutive_rejections)
    {
      consecutive_rejections_ = 0;
      return Decision::kJumpOverride;
    }
    return Decision::kRejectedJump;
  }

  void reset()
  {
    consecutive_rejections_ = 0;
  }

  int consecutive_rejections() const
  {
    return consecutive_rejections_;
  }

private:
  bool within_limits(const JumpMetrics & metrics) const
  {
    return metrics.translation <= config_.max_translation_jump &&
           metrics.yaw_deg <= config_.max_yaw_jump_deg;
  }

  Config config_;
  int consecutive_rejections_ = 0;
};

}  // namespace chassis_3d_localization

#endif  // CHASSIS_3D_LOCALIZATION__ALIGNMENT_GATE_HPP_
