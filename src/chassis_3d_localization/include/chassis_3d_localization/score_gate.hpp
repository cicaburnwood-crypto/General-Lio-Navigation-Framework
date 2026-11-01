#ifndef CHASSIS_3D_LOCALIZATION__SCORE_GATE_HPP_
#define CHASSIS_3D_LOCALIZATION__SCORE_GATE_HPP_

namespace chassis_3d_localization
{

// Quality gate for refine-stage RMS scores.
//
// The absolute rms floor of a correct pose moves day to day with scene-vs-map
// drift (measured 0.105 m with the room matching the map, 0.16 m after a few
// chairs were added), so a tight absolute threshold cannot transfer across
// days: it either blocks initialization outright or stops filtering. What
// does transfer is the relative feature - a wrong-mode solution scores ~25%
// worse than the same-day baseline. This gate therefore keeps a slow EMA
// baseline of accepted scores and rejects outliers relative to it. The
// configured ceiling is only an absolute sanity bound: scores above it are
// never acceptable, and persistent ceiling rejections mean the scene no
// longer matches the map.
//
// Like the jump gate, a purely relative check can deadlock: in a spot whose
// local floor is more than factor above the baseline, every solve is
// rejected while the baseline (updated only on accepts) never catches up.
// The Nth consecutive outlier rejection is therefore converted into an
// override that re-seeds the baseline. Ceiling rejections never count toward
// the override.
class ScoreGate
{
public:
  struct Config
  {
    double ceiling = 0.2;         // absolute sanity bound, meters RMS
    double outlier_factor = 1.2;  // reject above baseline * factor
    double baseline_beta = 0.2;   // EMA weight of the newest accepted rms
    // Consecutive outlier rejections converted into an override. <= 0
    // disables the override (reject until the baseline recovers by itself).
    int max_consecutive_rejections = 5;
  };

  enum class Decision
  {
    kAccepted,
    kScoreOverride,
    kRejectedCeiling,
    kRejectedOutlier,
  };

  ScoreGate() = default;

  explicit ScoreGate(const Config & config)
  : config_(config)
  {
  }

  Decision evaluate(const double rms)
  {
    if (rms > config_.ceiling) {
      return Decision::kRejectedCeiling;
    }
    if (!has_baseline_) {
      seed(rms);
      return Decision::kAccepted;
    }
    if (rms <= baseline_ * config_.outlier_factor) {
      baseline_ += config_.baseline_beta * (rms - baseline_);
      consecutive_rejections_ = 0;
      return Decision::kAccepted;
    }

    ++consecutive_rejections_;
    if (config_.max_consecutive_rejections > 0 &&
      consecutive_rejections_ >= config_.max_consecutive_rejections)
    {
      // The local floor moved (scene change, cluttered corner); re-baseline
      // there instead of stalling corrections indefinitely.
      seed(rms);
      return Decision::kScoreOverride;
    }
    return Decision::kRejectedOutlier;
  }

  void reset()
  {
    has_baseline_ = false;
    baseline_ = 0.0;
    consecutive_rejections_ = 0;
  }

  // -1.0 until the first solution has been accepted. Note the baseline is
  // self-healing: even if it is seeded by a wrong-mode solution, subsequent
  // better scores keep passing (they are below the cut) and pull it down.
  double baseline() const
  {
    return has_baseline_ ? baseline_ : -1.0;
  }

  int consecutive_rejections() const
  {
    return consecutive_rejections_;
  }

private:
  void seed(const double rms)
  {
    baseline_ = rms;
    has_baseline_ = true;
    consecutive_rejections_ = 0;
  }

  Config config_;
  bool has_baseline_ = false;
  double baseline_ = 0.0;
  int consecutive_rejections_ = 0;
};

}  // namespace chassis_3d_localization

#endif  // CHASSIS_3D_LOCALIZATION__SCORE_GATE_HPP_
