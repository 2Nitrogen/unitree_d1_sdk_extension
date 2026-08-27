// Subscriptions for the D1 arm, in the style of unitree/dds_wrapper/robots/go2.
#pragma once

#include <unitree/dds_wrapper/common/Subscription.h>

#include <unitree_arm/dds_wrapper/d1/d1_protocol.h>
#include <unitree_arm/idl/ArmString_.hpp>
#include <unitree_arm/idl/PubServoInfo_.hpp>

namespace unitree
{
namespace robot
{
namespace d1
{
namespace subscription
{

/// Receives commands on rt/arm_Command and decodes them.
///
/// This is the arm side of the link: a simulator or a custom controller
/// subscribes here to be driven exactly as the real arm is. `command` holds the
/// last message that parsed; `valid` is false while nothing usable has arrived.
class ArmCommand : public SubscriptionBase<unitree_arm::msg::dds_::ArmString_>
{
public:
  using SharedPtr = std::shared_ptr<ArmCommand>;

  ArmCommand(std::string topic = topic::kArmCommand)
  : SubscriptionBase<MsgType>(topic)
  {}

  Command command;
  bool valid{false};
  /// Raw payload of the last message, parsed or not -- useful when a peer
  /// sends a funcode this wrapper does not model.
  std::string raw;

private:
  void post_communication() override
  {
    raw = msg_.data_();
    valid = DecodeCommand(raw, command);
  }
};

/// Receives the arm's joint angles from current_servo_angle, in DEGREES.
class ServoAngle : public SubscriptionBase<unitree_arm::msg::dds_::PubServoInfo_>
{
public:
  using SharedPtr = std::shared_ptr<ServoAngle>;

  ServoAngle(std::string topic = topic::kServoAngle)
  : SubscriptionBase<MsgType>(topic)
  {}

  ServoAngles angles{};

private:
  void post_communication() override
  {
    angles[0] = msg_.servo0_data_();
    angles[1] = msg_.servo1_data_();
    angles[2] = msg_.servo2_data_();
    angles[3] = msg_.servo3_data_();
    angles[4] = msg_.servo4_data_();
    angles[5] = msg_.servo5_data_();
    angles[6] = msg_.servo6_data_();
  }
};

/// Receives the JSON feedback stream (angles, status, acknowledgements).
///
/// Note the topic default: the arm publishes on rt/arm_Feedback. The stock SDK
/// example subscribes to arm_Feedback, which never fires against real hardware.
class ArmFeedback : public SubscriptionBase<unitree_arm::msg::dds_::ArmString_>
{
public:
  using SharedPtr = std::shared_ptr<ArmFeedback>;

  ArmFeedback(std::string topic = topic::kArmFeedback)
  : SubscriptionBase<MsgType>(topic)
  {}

  Feedback feedback;
  bool valid{false};
  std::string raw;

  /// Last angle report seen, regardless of what the newest message carried.
  ServoAngles angles{};
  bool has_angles{false};

private:
  void post_communication() override
  {
    raw = msg_.data_();
    valid = DecodeFeedback(raw, feedback);
    if (valid && feedback.has_angles) {
      angles = feedback.angles;
      has_angles = true;
    }
  }
};

}  // namespace subscription
}  // namespace d1
}  // namespace robot
}  // namespace unitree
