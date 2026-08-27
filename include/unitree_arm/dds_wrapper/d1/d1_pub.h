// Publishers for the D1 arm, in the style of unitree/dds_wrapper/robots/go2.
#pragma once

#include <unitree/dds_wrapper/common/Publisher.h>

#include <unitree_arm/dds_wrapper/d1/d1_protocol.h>
#include <unitree_arm/idl/ArmString_.hpp>
#include <unitree_arm/idl/PubServoInfo_.hpp>

#include <atomic>

namespace unitree
{
namespace robot
{
namespace d1
{
namespace publisher
{

/// Sends commands on rt/arm_Command.
///
/// Use it from the host side to drive the arm. `seq` is generated here; the
/// guide only requires that a reply can be matched back to its command.
class ArmCommand : public RealTimePublisher<unitree_arm::msg::dds_::ArmString_>
{
public:
  using SharedPtr = std::shared_ptr<ArmCommand>;

  ArmCommand(std::string topic = topic::kArmCommand)
  : RealTimePublisher<MsgType>(topic)
  {}

  /// Every Send* returns false when the message could not be built -- today
  /// that means an inf or NaN angle. Nothing goes on the wire in that case.

  /// funcode 1
  bool SendJointAngle(uint8_t id, float angle_deg, int16_t delay_ms = 0)
  { return Send(EncodeJointAngle(NextSeq(), id, angle_deg, delay_ms)); }

  /// funcode 2
  bool SendAllJointAngle(const ServoAngles& angle_deg,
                         int mode = smoothing::kTrajectory)
  { return Send(EncodeAllJointAngle(NextSeq(), angle_deg, mode)); }

  /// funcode 4
  bool SendJointEnable(uint8_t id, int mode)
  { return Send(EncodeJointEnable(NextSeq(), id, mode)); }

  /// funcode 5. mode runs 0 (released) .. kStiffnessMax (locked).
  bool SendAllJointEnable(int mode)
  { return Send(EncodeAllJointEnable(NextSeq(), mode)); }

  /// funcode 6
  bool SendPower(int power) { return Send(EncodePower(NextSeq(), power)); }

  /// funcode 7
  bool SendZero() { return Send(EncodeZero(NextSeq())); }

  /// Escape hatch for a payload this wrapper does not model yet.
  bool Send(const std::string& json)
  {
    if (json.empty()) return false;
    this->lock();
    this->msg_.data_(json);
    this->unlockAndPublish();
    return true;
  }

  int32_t NextSeq() { return ++seq_; }

private:
  std::atomic<int32_t> seq_{0};
};

/// Publishes joint angles on current_servo_angle at the arm's 10 Hz cycle.
/// Values are DEGREES, servo J0..J6.
class ServoAngle : public RealTimePublisher<unitree_arm::msg::dds_::PubServoInfo_>
{
public:
  using SharedPtr = std::shared_ptr<ServoAngle>;

  ServoAngle(std::string topic = topic::kServoAngle)
  : RealTimePublisher<MsgType>(topic)
  {}

  void Send(const ServoAngles& angle_deg)
  {
    // PubServoInfo_ is plain floats, so there is no malformed-JSON hazard here.
    this->lock();
    this->msg_.servo0_data_(angle_deg[0]);
    this->msg_.servo1_data_(angle_deg[1]);
    this->msg_.servo2_data_(angle_deg[2]);
    this->msg_.servo3_data_(angle_deg[3]);
    this->msg_.servo4_data_(angle_deg[4]);
    this->msg_.servo5_data_(angle_deg[5]);
    this->msg_.servo6_data_(angle_deg[6]);
    this->unlockAndPublish();
  }
};

/// Publishes the JSON feedback stream.
///
/// The arm's driver uses rt/arm_Feedback, but the stock example
/// get_arm_joint_angle.cpp listens on arm_Feedback. A service standing in for
/// the arm should construct one of these per topic so either client works.
class ArmFeedback : public RealTimePublisher<unitree_arm::msg::dds_::ArmString_>
{
public:
  using SharedPtr = std::shared_ptr<ArmFeedback>;

  ArmFeedback(std::string topic = topic::kArmFeedback)
  : RealTimePublisher<MsgType>(topic)
  {}

  bool Send(const std::string& json)
  {
    if (json.empty()) return false;
    this->lock();
    this->msg_.data_(json);
    this->unlockAndPublish();
    return true;
  }

  bool SendAngles(const ServoAngles& angle_deg)
  { return Send(EncodeAngleFeedback(angle_deg)); }

  bool SendStatus(int enable_status, int power_status, int error_status)
  { return Send(EncodeStatusFeedback(enable_status, power_status, error_status)); }

  bool SendMotorStatus(const std::array<int, kNumServo>& status)
  { return Send(EncodeMotorStatus(status)); }

  bool SendRecvAck(int32_t seq, int recv_status)
  { return Send(EncodeRecvAck(seq, recv_status)); }

  bool SendExecAck(int32_t seq, int exec_status)
  { return Send(EncodeExecAck(seq, exec_status)); }
};

}  // namespace publisher
}  // namespace d1
}  // namespace robot
}  // namespace unitree
