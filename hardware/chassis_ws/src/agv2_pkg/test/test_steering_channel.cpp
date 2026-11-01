#include <gtest/gtest.h>
#include <cmath>
#include <map>
#include <vector>
#include "agv2_pkg/steering_channel.hpp"
using namespace agv2_pkg;

struct FakeMotor {
  SteeringChannel channel;
  int64_t time{0};
  double angle{0};
  uint8_t error{0};
  std::map<uint8_t, int> counts;
  std::vector<SteeringRequest> sent;
  std::optional<SteeringRequest> response;
  int64_t response_at{0};
  void run(int duration, bool permit=true, bool replies=true, bool enable_reply=true) {
    for (int i=0; i<duration; ++i, ++time) {
      if (response && time == response_at) {
        if (replies && (enable_reply || response->command != 0x88)) {
          if (response->command == 0xa4) angle = response->angle_rad;
          channel.receive(response->command, time, angle, error);
        }
        response.reset();
      }
      auto request = channel.poll(time, permit);
      if (request) {
        EXPECT_FALSE(response.has_value());
        EXPECT_TRUE(channel.in_flight());
        ++counts[request->command];
        sent.push_back(*request);
        response = request;
        response_at = time + 2;
      }
    }
  }
};

TEST(SteeringChannel, PreflightDoesNotActuateEvenWithTargets) {
  FakeMotor m;
  m.channel.target(1.0);
  m.run(2000, false);
  EXPECT_EQ(m.counts[0x88], 0);
  EXPECT_EQ(m.counts[0xa4], 0);
  EXPECT_GE(m.counts[0x94], 39);
  EXPECT_FALSE(m.channel.ready(m.time));
}
TEST(SteeringChannel, SerializesAndPreserves20HzWhenHealthy) {
  FakeMotor m;
  m.channel.target(0.5);
  m.run(2000);
  EXPECT_TRUE(m.channel.ready(m.time));
  EXPECT_EQ(m.counts[0x88], 1);
  EXPECT_GE(m.counts[0xa4], 36);
  EXPECT_LE(m.counts[0xa4], 40);
  EXPECT_GE(m.counts[0x94], 39);
  EXPECT_LE(m.counts[0x9a], 2);
  EXPECT_DOUBLE_EQ(m.angle, 0.5);
}
TEST(SteeringChannel, AcceptsTrueZeroDegrees) {
  SteeringChannel ch;
  ASSERT_EQ(ch.poll(0, false)->command, 0x94);
  EXPECT_TRUE(ch.receive(0x94, 1, 0.0));
  EXPECT_DOUBLE_EQ(ch.measured(), 0.0);
  EXPECT_FALSE(ch.in_flight());
}
TEST(SteeringChannel, IgnoresWrongAndLateRepliesAndInvalidAngles) {
  SteeringChannel ch;
  ch.poll(0, false);
  EXPECT_FALSE(ch.receive(0xa4, 1));
  EXPECT_FALSE(ch.receive(0x94, 1, NAN));
  EXPECT_TRUE(ch.in_flight());
  EXPECT_FALSE(ch.receive(0x94, 20));
  EXPECT_FALSE(ch.poll(20, false));
  EXPECT_EQ(ch.timeouts(), 1U);
  EXPECT_FALSE(ch.poll(119, false));
  EXPECT_EQ(ch.poll(120, false)->command, 0x94);
}
TEST(SteeringChannel, LegacyEnableWithoutAckVerifiedByPositionAndCommand) {
  FakeMotor m;
  m.run(1000, true, true, false);
  EXPECT_TRUE(m.channel.ready(m.time));
  EXPECT_EQ(m.counts[0x88], 1);
}
TEST(SteeringChannel, ShortPacketLossDoesNotImmediatelyHold) {
  FakeMotor m;
  m.run(500);
  ASSERT_TRUE(m.channel.ready(m.time));
  m.run(80, true, false);
  EXPECT_TRUE(m.channel.ready(m.time));
  m.run(300);
  EXPECT_TRUE(m.channel.ready(m.time));
}
TEST(SteeringChannel, LossRecoversAndReacquiresBeforeResuming) {
  FakeMotor m;
  m.channel.target(0.5);
  m.run(500);
  m.run(700, true, false);
  ASSERT_FALSE(m.channel.ready(m.time));
  EXPECT_STREQ(m.channel.phase(m.time), "recovering");
  const int old_enables=m.counts[0x88];
  m.angle=-0.3;  // sensor/motor may have changed during outage
  m.run(1600);
  EXPECT_TRUE(m.channel.ready(m.time));
  EXPECT_EQ(m.counts[0x88], old_enables+1);
}
TEST(SteeringChannel, ExhaustedRecoveryDoesNotFloodOrEnableForever) {
  FakeMotor m;
  m.run(500);
  m.run(12000, true, false);
  EXPECT_STREQ(m.channel.phase(m.time), "recovery_exhausted_operator_retry_required");
  const auto count=m.sent.size();
  m.run(2000, true, false);
  EXPECT_EQ(m.sent.size(), count);
  m.channel.retry(m.time);
  m.run(1600);
  EXPECT_TRUE(m.channel.ready(m.time));
}
TEST(SteeringChannel, IdleCannotTriggerMovementOrRepeatedEnables) {
  FakeMotor m;
  m.run(500);
  const auto enables=m.counts[0x88], targets=m.counts[0xa4];
  m.run(15000, false);
  EXPECT_EQ(m.counts[0x88], enables);
  EXPECT_EQ(m.counts[0xa4], targets);
  EXPECT_STRNE(m.channel.phase(m.time), "recovery_exhausted_operator_retry_required");
}
TEST(SteeringChannel, FaultIsNotAutoClearedOrReEnabled) {
  FakeMotor m;
  m.run(500);
  m.error=8;
  m.run(700);
  ASSERT_FALSE(m.channel.ready(m.time));
  EXPECT_STREQ(m.channel.phase(m.time), "hardware_fault_latched");
  const auto enables=m.counts[0x88];
  m.error=0;
  m.run(2000);
  EXPECT_FALSE(m.channel.ready(m.time));
  EXPECT_EQ(m.counts[0x88], enables);
  m.channel.retry(m.time);
  m.run(1500);
  EXPECT_TRUE(m.channel.ready(m.time));
}
TEST(SteeringChannel, CoalescesTargetsInsteadOfReplayingQueue) {
  FakeMotor m;
  m.run(500);
  for (int i=0;i<1000;++i) m.channel.target(i / 1000.0);
  m.run(100);
  EXPECT_DOUBLE_EQ(m.angle, .999);
}
TEST(SteeringChannel, BadConfigurationRejected) {
  SteeringConfig c;
  c.response_ms=100;
  EXPECT_THROW(SteeringChannel{c}, std::invalid_argument);
}
