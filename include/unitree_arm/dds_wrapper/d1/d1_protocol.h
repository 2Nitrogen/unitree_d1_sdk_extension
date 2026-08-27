// Encode / decode for the D1 arm's JSON-over-DDS protocol.
//
// Every D1 message is an ArmString_ whose single field is a JSON document:
//   {"seq":.., "address":.., "funcode":.., "data":{..}}
// These helpers keep that format in one place so callers work with typed
// structs instead of hand-built strings.
//
// Angles are DEGREES on the wire, matching the developer guide and the SDK
// examples. For the gripper (J6) the wire value is the servo angle, which maps
// to jaw opening through a slider-crank linkage -- convert outside this layer.
//
// The Encode* functions that carry angles return an EMPTY STRING if any value
// is inf or NaN: those have no JSON form, and a malformed command is worse than
// no command. The publishers in d1_pub.h drop empty payloads rather than put
// them on the wire, so a diverged controller cannot make the arm act on
// garbage. Check the return value if you need to detect it.
#pragma once

#include <unitree_arm/dds_wrapper/d1/defines.h>

#include <array>
#include <cstdint>
#include <string>

namespace unitree
{
namespace robot
{
namespace d1
{

using ServoAngles = std::array<float, kNumServo>;

// --------------------------------------------------------------- commands
/// funcode 1: move one servo. delay_ms is the execution time; the guide's
/// examples leave it at 0.
std::string EncodeJointAngle(int32_t seq, uint8_t id, float angle_deg,
                             int16_t delay_ms = 0);

/// funcode 2: move all seven servos in one message.
std::string EncodeAllJointAngle(int32_t seq, const ServoAngles& angle_deg,
                                int mode = smoothing::kTrajectory);

/// funcode 4: enable (1) or release (0) one servo.
std::string EncodeJointEnable(int32_t seq, uint8_t id, int mode);

/// funcode 5: enable/release every servo. mode runs 0 (fully released) to
/// kStiffnessMax (fully locked).
std::string EncodeAllJointEnable(int32_t seq, int mode);

/// funcode 6: motor power switch. Doubles as an emergency stop.
std::string EncodePower(int32_t seq, int power);

/// funcode 7: return every servo to its zero position.
std::string EncodeZero(int32_t seq);

// -------------------------------------------------------------- feedback
/// funcode 1 under address 2: the periodic angle report.
std::string EncodeAngleFeedback(const ServoAngles& angle_deg,
                                int32_t seq = kFeedbackSeq);

/// funcode 3 under address 2. error_status is 1 for normal, 0 for a fault.
std::string EncodeStatusFeedback(int enable_status, int power_status,
                                 int error_status, int32_t seq = kFeedbackSeq);

/// funcode 4 under address 2: per-motor online flags, 1 = healthy.
std::string EncodeMotorStatus(const std::array<int, kNumServo>& status,
                              int32_t seq = kFeedbackSeq);

/// address 3: acknowledgements. recv_status/exec_status are 1 on success.
std::string EncodeRecvAck(int32_t seq, int recv_status);
std::string EncodeExecAck(int32_t seq, int exec_status);

// --------------------------------------------------------------- parsing
/// A decoded address-1 message. Only the fields belonging to `funcode` are
/// meaningful; `has_angle` marks which entries of `angles` the sender supplied.
struct Command
{
  int32_t seq{0};
  int funcode{0};

  uint8_t id{0};          ///< funcode 1, 4
  float angle_deg{0.f};   ///< funcode 1
  int16_t delay_ms{0};    ///< funcode 1

  ServoAngles angles{};                    ///< funcode 2
  std::array<bool, kNumServo> has_angle{};  ///< funcode 2
  int mode{smoothing::kTrajectory};        ///< funcode 2

  int enable_mode{0};  ///< funcode 4, 5
  int power{0};        ///< funcode 6
};

/// A decoded address-2 or address-3 message.
struct Feedback
{
  int32_t seq{0};
  int address{0};
  int funcode{0};

  ServoAngles angles{};   ///< address 2 / funcode 1
  bool has_angles{false};

  int enable_status{0};   ///< address 2 / funcode 3
  int power_status{0};
  int error_status{0};
  bool has_status{false};

  std::array<int, kNumServo> motor_status{};  ///< address 2 / funcode 4
  bool has_motor_status{false};

  int recv_status{0};  ///< address 3 / funcode 1
  int exec_status{0};  ///< address 3 / funcode 2
  bool has_ack{false};
};

/// Parse a message addressed to the arm. Returns false on malformed JSON or a
/// different address, leaving `out` untouched.
bool DecodeCommand(const std::string& json, Command& out);

/// Parse a message coming back from the arm (address 2 or 3).
bool DecodeFeedback(const std::string& json, Feedback& out);

}  // namespace d1
}  // namespace robot
}  // namespace unitree
