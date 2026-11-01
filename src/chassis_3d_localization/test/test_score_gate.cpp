#include <gtest/gtest.h>

#include "chassis_3d_localization/score_gate.hpp"

namespace
{

using chassis_3d_localization::ScoreGate;
using Decision = ScoreGate::Decision;

ScoreGate::Config make_config()
{
  ScoreGate::Config config;
  config.ceiling = 0.2;
  config.outlier_factor = 1.2;
  config.baseline_beta = 0.2;
  config.max_consecutive_rejections = 5;
  return config;
}

}  // namespace

TEST(ScoreGate, FirstQualifiedSolutionSeedsBaseline)
{
  ScoreGate gate(make_config());
  EXPECT_DOUBLE_EQ(gate.baseline(), -1.0);
  EXPECT_EQ(gate.evaluate(0.10), Decision::kAccepted);
  EXPECT_DOUBLE_EQ(gate.baseline(), 0.10);
}

TEST(ScoreGate, CeilingRejectsAndNeverCountsTowardOverride)
{
  ScoreGate gate(make_config());
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(gate.evaluate(0.25), Decision::kRejectedCeiling);
  }
  EXPECT_EQ(gate.consecutive_rejections(), 0);
  EXPECT_DOUBLE_EQ(gate.baseline(), -1.0);
}

TEST(ScoreGate, AcceptedSolutionUpdatesBaselineEma)
{
  ScoreGate gate(make_config());
  gate.evaluate(0.10);
  EXPECT_EQ(gate.evaluate(0.12), Decision::kAccepted);  // 0.12 <= 0.10 * 1.2
  EXPECT_NEAR(gate.baseline(), 0.104, 1e-9);            // 0.10 + 0.2 * (0.12 - 0.10)
}

TEST(ScoreGate, FieldBadYawModeIsRejectedAsOutlier)
{
  // 07-07 field data: good-mode floor 0.105, 4deg-off mode 0.132 (ratio 1.26).
  ScoreGate gate(make_config());
  gate.evaluate(0.105);
  EXPECT_EQ(gate.evaluate(0.132), Decision::kRejectedOutlier);
  EXPECT_EQ(gate.consecutive_rejections(), 1);
}

TEST(ScoreGate, AcceptResetsRejectionStreak)
{
  ScoreGate gate(make_config());
  gate.evaluate(0.10);
  gate.evaluate(0.15);
  gate.evaluate(0.15);
  EXPECT_EQ(gate.consecutive_rejections(), 2);
  EXPECT_EQ(gate.evaluate(0.11), Decision::kAccepted);
  EXPECT_EQ(gate.consecutive_rejections(), 0);
}

TEST(ScoreGate, OverrideAfterNRejectionsReseedsBaseline)
{
  ScoreGate gate(make_config());
  gate.evaluate(0.09);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(gate.evaluate(0.13), Decision::kRejectedOutlier);
  }
  EXPECT_EQ(gate.evaluate(0.13), Decision::kScoreOverride);
  EXPECT_DOUBLE_EQ(gate.baseline(), 0.13);
  EXPECT_EQ(gate.consecutive_rejections(), 0);
  // The refreshed baseline accepts the local floor from now on.
  EXPECT_EQ(gate.evaluate(0.13), Decision::kAccepted);
}

TEST(ScoreGate, BaselineSelfHealsAfterBadSeed)
{
  // Seeded by a wrong-mode solution: better scores keep passing and pull the
  // baseline back down.
  ScoreGate gate(make_config());
  gate.evaluate(0.132);
  EXPECT_EQ(gate.evaluate(0.105), Decision::kAccepted);
  EXPECT_LT(gate.baseline(), 0.132);
}

TEST(ScoreGate, NonPositiveLimitDisablesOverride)
{
  auto config = make_config();
  config.max_consecutive_rejections = 0;
  ScoreGate gate(config);
  gate.evaluate(0.09);
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(gate.evaluate(0.13), Decision::kRejectedOutlier);
  }
}

TEST(ScoreGate, ResetClearsBaselineAndStreak)
{
  ScoreGate gate(make_config());
  gate.evaluate(0.10);
  gate.evaluate(0.15);
  gate.reset();
  EXPECT_DOUBLE_EQ(gate.baseline(), -1.0);
  EXPECT_EQ(gate.consecutive_rejections(), 0);
  EXPECT_EQ(gate.evaluate(0.18), Decision::kAccepted);  // reseeds under ceiling
  EXPECT_DOUBLE_EQ(gate.baseline(), 0.18);
}
