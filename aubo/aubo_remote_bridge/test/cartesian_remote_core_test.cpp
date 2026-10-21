#include <gtest/gtest.h>
#include "aubo_remote_bridge/jog_remote_core.hpp"
namespace abr = aubo_remote_bridge;
TEST(JogProtocol, CCodecAndParserRecovery) {
  const uint8_t vector[] = {'1','2','3','4','5','6','7','8','9'};
  EXPECT_EQ(Remote_CRC(vector, 9), 0x29b1);
  RemoteJog_Input input{}; input.session = 0x12345678; input.request_id = 42;
  input.command = REMOTE_JOG_HOME; input.flags = 3; input.speed = 1000; input.axis[2] = -1000;
  abr::Frame frame{}; ASSERT_TRUE(RemoteJog_EncodeInput(&input, 7, &frame));
  Remote_ParserTypeDef parser{}; Remote_ParserInit(&parser);
  auto bytes = abr::encode(frame); auto bad = bytes; bad.back() ^= 0x40;
  abr::Frame parsed{}; int frames = 0;
  for (auto b : bad) frames += Remote_ParseByte(&parser, b, &parsed);
  EXPECT_EQ(frames, 0);
  for (auto b : bytes) frames += Remote_ParseByte(&parser, b, &parsed);
  EXPECT_EQ(frames, 1);
  RemoteJog_Input output{}; ASSERT_TRUE(RemoteJog_DecodeInput(&parsed, &output));
  EXPECT_EQ(output.session, input.session); EXPECT_EQ(output.axis[2], -1000);
  input.axis[2] = 1001; EXPECT_FALSE(RemoteJog_EncodeInput(&input, 7, &frame));
  input.axis[2] = 1000; input.speed = 0; EXPECT_FALSE(RemoteJog_EncodeInput(&input, 7, &frame));
  input.speed = 1001; EXPECT_FALSE(RemoteJog_EncodeInput(&input, 7, &frame));
  input.speed = 100; input.frame = 2; EXPECT_FALSE(RemoteJog_EncodeInput(&input, 7, &frame));
}
TEST(JogProtocol, FeedbackRoundTrip) {
  RemoteJog_Feedback input{}; input.session = 12; input.request_id = 44;
  input.controller_state = REMOTE_JOG_HOMING; input.home_state = 1; input.joint_mask = 63;
  input.joints[0] = -3040; input.home_error = 65535; input.fault = 5;
  abr::Frame frame{}; RemoteJog_EncodeFeedback(&input, 17, &frame);
  EXPECT_EQ(frame.Length, 28);
  RemoteJog_Feedback output{}; ASSERT_TRUE(RemoteJog_DecodeFeedback(&frame, &output));
  EXPECT_EQ(output.joints[0], -3040); EXPECT_EQ(output.home_error, 65535); EXPECT_EQ(output.fault, 5);
}
TEST(JogProtocol, PollFreshnessAndSessionRotation) {
  abr::JogPoller poller(100); abr::Frame poll{}, frame{}; RemoteJog_Input input{}, output{};
  ASSERT_TRUE(poller.poll(1, poll)); input.session = 100; input.flags = 3;
  ASSERT_TRUE(RemoteJog_EncodeInput(&input, poll.Sequence, &frame));
  auto wrong = frame; wrong.Sequence++;
  EXPECT_FALSE(poller.receive(wrong, 1.01, output));
  wrong = frame; Remote_PutU32(wrong.Payload, 99);
  EXPECT_FALSE(poller.receive(wrong, 1.01, output));
  EXPECT_TRUE(poller.receive(frame, 1.01, output));
  EXPECT_FALSE(poller.receive(frame, 1.011, output));  // 重复响应不刷新看门狗
  EXPECT_TRUE(poller.online(1.02));
  ASSERT_TRUE(poller.poll(1.03, poll));
  EXPECT_FALSE(poller.receive(frame, 1.04, output));
  ASSERT_TRUE(poller.poll(1.14, poll)); EXPECT_EQ(poller.session(), 101u);
  EXPECT_FALSE(poller.online(1.14));
  EXPECT_FALSE(poller.receive(frame, 1.15, output));
  // 原HOME/ZERO INPUT的类型与长度不能成为有效点动输入。
  frame.Type = REMOTE_INPUT; frame.Length = 18;
  EXPECT_FALSE(RemoteJog_DecodeInput(&frame, &output));
}
TEST(JogProtocol, RecalibrationRotatesSessionWithoutSerialTimeout) {
  abr::JogPoller poller(100); abr::Frame poll{}, frame{}; RemoteJog_Input input{}, output{};
  ASSERT_TRUE(poller.poll(1, poll)); input.session = 100; input.flags = 3;
  input.request_id = 5; input.command = REMOTE_JOG_ENABLE;
  ASSERT_TRUE(RemoteJog_EncodeInput(&input, poll.Sequence, &frame));
  ASSERT_TRUE(poller.receive(frame, 1.001, output));
  ASSERT_TRUE(poller.poll(1.03, poll));
  input.flags = 0; input.request_id = 0; input.command = 0;
  ASSERT_TRUE(RemoteJog_EncodeInput(&input, poll.Sequence, &frame));
  ASSERT_TRUE(poller.receive(frame, 1.031, output));
  EXPECT_EQ(poller.session(), 100u);  // 先回复本次无效输入，再于下一次POLL换会话。
  ASSERT_TRUE(poller.poll(1.06, poll));
  EXPECT_EQ(poller.session(), 101u); EXPECT_EQ(poll.Sequence, 1);
  EXPECT_FALSE(poller.receive(frame, 1.061, output));  // 旧会话反馈不能混入新会话。
  input.session = 101;
  ASSERT_TRUE(RemoteJog_EncodeInput(&input, poll.Sequence, &frame));
  ASSERT_TRUE(poller.receive(frame, 1.062, output));
  ASSERT_TRUE(poller.poll(1.09, poll));
  EXPECT_EQ(poller.session(), 101u);  // 校准未完成时不反复换会话。
  input.flags = 3;
  ASSERT_TRUE(RemoteJog_EncodeInput(&input, poll.Sequence, &frame));
  ASSERT_TRUE(poller.receive(frame, 1.091, output));
  ASSERT_TRUE(poller.poll(1.12, poll));
  input.request_id = 1; input.command = REMOTE_JOG_ENABLE;
  ASSERT_TRUE(RemoteJog_EncodeInput(&input, poll.Sequence, &frame));
  ASSERT_TRUE(poller.receive(frame, 1.121, output));
  EXPECT_EQ(output.request_id, 1); EXPECT_EQ(output.session, 101u);
}
TEST(JogEnable, RequiresNewIdleStateAfterServiceReturns) {
  abr::JogEnableConfirmation enable;
  enable.begin(10, 1.0);
  EXPECT_EQ(enable.update(1.01, 10, true, true), abr::JogEnableConfirmation::Waiting);
  EXPECT_EQ(enable.update(1.03, 11, false, true), abr::JogEnableConfirmation::Waiting);
  EXPECT_TRUE(enable.pending());
  EXPECT_EQ(enable.update(1.10, 12, true, true), abr::JogEnableConfirmation::Confirmed);
  EXPECT_FALSE(enable.pending());
}
TEST(JogEnable, InputLossTimeoutAndCancellationCannotRearm) {
  abr::JogEnableConfirmation enable;
  enable.begin(10, 1.0);
  EXPECT_EQ(enable.update(1.01, 11, true, false), abr::JogEnableConfirmation::Failed);
  EXPECT_EQ(enable.update(1.02, 12, true, true), abr::JogEnableConfirmation::Waiting);
  EXPECT_FALSE(enable.pending());
  enable.begin(20, 2.0);
  EXPECT_EQ(enable.update(3.0, 21, true, true), abr::JogEnableConfirmation::Failed);
  enable.begin(30, 4.0); enable.cancel();
  EXPECT_EQ(enable.update(4.1, 31, true, true), abr::JogEnableConfirmation::Waiting);
  EXPECT_FALSE(enable.pending());
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
