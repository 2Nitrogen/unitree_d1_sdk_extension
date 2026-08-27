// Wire-protocol constants for the Unitree D1 arm service.
//
// Source: D1 developer guide (support.unitree.com/home/en/developer/D1Arm_services)
// and the arm's own driver, marm_communication_node.cpp.
#pragma once

#include <cstdint>

namespace unitree
{
namespace robot
{
namespace d1
{

/// 6 joints + 1 gripper. The guide numbers them from the base: J0 .. J6,
/// where J6 is the gripper. One servo per index -- the CAD/URDF splits the
/// gripper into two prismatic fingers, but the hardware drives them with a
/// single servo.
constexpr int kNumServo = 7;
constexpr int kGripperIndex = 6;

namespace topic
{
/// Commands to the arm.
constexpr const char* kArmCommand = "rt/arm_Command";
/// Joint angle feedback, PubServoInfo_, 10 Hz.
constexpr const char* kServoAngle = "current_servo_angle";
/// Feedback as JSON. The arm's driver publishes on "rt/arm_Feedback"; the
/// stock SDK example get_arm_joint_angle.cpp subscribes to "arm_Feedback"
/// instead, so a service impersonating the arm should publish on both.
constexpr const char* kArmFeedback = "rt/arm_Feedback";
constexpr const char* kArmFeedbackAlt = "arm_Feedback";
}  // namespace topic

/// "address" field: which side the message is addressed to.
namespace address
{
constexpr int kCommand = 1;   ///< host -> arm
constexpr int kFeedback = 2;  ///< arm -> host, periodic state
constexpr int kAck = 3;       ///< arm -> host, per-command acknowledgement
}  // namespace address

/// "funcode" field, interpreted within an address.
namespace funcode
{
// address 1
constexpr int kJointAngle = 1;      ///< {"id", "angle", "delay_ms"}
constexpr int kAllJointAngle = 2;   ///< {"mode", "angle0".."angle6"}
constexpr int kJointEnable = 4;     ///< {"id", "mode"}
constexpr int kAllJointEnable = 5;  ///< {"mode"} 0 = released .. 80000 = locked
constexpr int kPower = 6;           ///< {"power"} 0 = off, 1 = on
constexpr int kZero = 7;            ///< no data
// address 2
constexpr int kAngleFeedback = 1;   ///< {"angle0".."angle6"}
constexpr int kStatusFeedback = 3;  ///< {"enable_status","power_status","error_status"}
constexpr int kMotorStatus = 4;     ///< {"motor0_status".."motor6_status"}
// address 3
constexpr int kRecvAck = 1;         ///< {"recv_status"}
constexpr int kExecAck = 2;         ///< {"exec_status"}
}  // namespace funcode

/// funcode 5 "mode" range: 0 fully released, 80000 fully locked.
constexpr int kStiffnessMax = 80000;

/// Smoothing mode of funcode 2. The guide describes 0 as "small smoothing of
/// 10Hz data" and 1 as "large smoothing of trajectory-use".
namespace smoothing
{
constexpr int kRealtime = 0;
constexpr int kTrajectory = 1;
}  // namespace smoothing

/// Active (unsolicited) messages carry a fixed seq of 10; replies echo the
/// seq of the command they answer.
constexpr int32_t kFeedbackSeq = 10;

/// The arm's control and feedback cycle.
constexpr double kControlRateHz = 10.0;

}  // namespace d1
}  // namespace robot
}  // namespace unitree
