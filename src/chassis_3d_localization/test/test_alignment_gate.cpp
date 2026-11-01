#include <gtest/gtest.h>

#include "chassis_3d_localization/alignment_gate.hpp"

namespace
{

using chassis_3d_localization::AlignmentGate;

AlignmentGate::Config make_config()
{
  AlignmentGate::Config config;
  config.max_translation_jump = 0.3;
  config.max_yaw_jump_deg = 5.0;
  config.max_consecutive_rejections = 3;
  return config;
}

constexpr AlignmentGate::JumpMetrics kSmallJump{0.05, 0.5};
constexpr AlignmentGate::JumpMetrics kBigTranslationJump{0.6, 0.5};
constexpr AlignmentGate::JumpMetrics kBigYawJump{0.05, 8.0};

}  // namespace

TEST(AlignmentGate, AcceptsWithinLimits)
{
  AlignmentGate gate(make_config());
  EXPECT_EQ(gate.evaluate(kSmallJump, false), AlignmentGate::Decision::kAccepted);
  EXPECT_EQ(gate.consecutive_rejections(), 0);
}

TEST(AlignmentGate, RejectsTranslationJump)
{
  AlignmentGate gate(make_config());
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
  EXPECT_EQ(gate.consecutive_rejections(), 1);
}

TEST(AlignmentGate, RejectsYawJump)
{
  AlignmentGate gate(make_config());
  EXPECT_EQ(gate.evaluate(kBigYawJump, false), AlignmentGate::Decision::kRejectedJump);
  EXPECT_EQ(gate.consecutive_rejections(), 1);
}

TEST(AlignmentGate, OverridesAfterConsecutiveRejections)
{
  AlignmentGate gate(make_config());
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kJumpOverride);
  EXPECT_EQ(gate.consecutive_rejections(), 0);
}

TEST(AlignmentGate, AcceptResetsRejectionCount)
{
  AlignmentGate gate(make_config());
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
  EXPECT_EQ(gate.evaluate(kSmallJump, false), AlignmentGate::Decision::kAccepted);
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
  EXPECT_EQ(gate.consecutive_rejections(), 1);
}

TEST(AlignmentGate, BypassSkipsJumpCheckAndResetsCount)
{
  AlignmentGate gate(make_config());
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, true), AlignmentGate::Decision::kAccepted);
  EXPECT_EQ(gate.consecutive_rejections(), 0);
}

TEST(AlignmentGate, ResetClearsRejectionCount)
{
  AlignmentGate gate(make_config());
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
  gate.reset();
  EXPECT_EQ(gate.consecutive_rejections(), 0);
  EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
}

TEST(AlignmentGate, NonPositiveLimitDisablesOverride)
{
  auto config = make_config();
  config.max_consecutive_rejections = 0;
  AlignmentGate gate(config);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(gate.evaluate(kBigTranslationJump, false), AlignmentGate::Decision::kRejectedJump);
  }
  EXPECT_EQ(gate.consecutive_rejections(), 10);
}
