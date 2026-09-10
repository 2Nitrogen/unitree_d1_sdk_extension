// Serve a D1 arm that lives inside a MuJoCo model.
//
// This header stands in for the arm's own controller (marm_communication_node):
// it subscribes to rt/arm_Command, drives the arm's actuators in mj_data->ctrl,
// and reports joint angles on current_servo_angle and rt/arm_Feedback at the
// hardware's 10 Hz cycle. A program written against the D1 SDK therefore drives
// the simulator without a single change.
//
// It is a header-only add-on: including it pulls in MuJoCo, so a project that
// does not simulate never pays for it and libunitree_arm has no MuJoCo
// dependency of its own.
//
// Wiring it into unitree_mujoco takes four lines -- see the "Simulation" section
// of the README.
//
//   #include <unitree_arm/dds_wrapper/d1/d1_mujoco_bridge.h>
//
//   // in UnitreeSDK2BridgeBase::_check_sensor(), before dim_motor_sensor_ is set
//   arm_ = unitree::robot::d1::MujocoArmLayout::Detect(mj_model_);
//   if (arm_.valid()) num_motor_ = arm_.base;
//
//   // in main(), after interface->start()
//   auto arm_bridge = unitree::robot::d1::StartMujocoArmBridge(m, d);
//
// Both halves are no-ops on a model without an arm, so every other robot keeps
// its original behaviour.
#pragma once

#include <mujoco/mujoco.h>

#include <unitree/common/thread/recurrent_thread.hpp>
#include <unitree_arm/dds_wrapper/d1/d1.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

namespace unitree
{
namespace robot
{
namespace d1
{

/// Name of the arm's first actuator. Detection keys off this rather than the
/// robot's name, so any scene that mounts a D1 is picked up automatically.
constexpr const char* kFirstActuatorName = "d1_J0";

/// Where the arm sits in mj_data->ctrl.
struct MujocoArmLayout
{
  int base = -1;  ///< ctrl index of servo J0, or -1 when the model has no arm
  int count = 0;  ///< number of arm servos present

  bool valid() const { return base >= 0 && count > 0; }

  static MujocoArmLayout Detect(const mjModel* m)
  {
    MujocoArmLayout out;
    if (!m) return out;
    const int a0 = mj_name2id(m, mjOBJ_ACTUATOR, kFirstActuatorName);
    if (a0 < 0) return out;
    out.base = a0;
    out.count = std::min(kNumServo, m->nu - a0);
    return out;
  }
};

/// Drives the arm actuators of a MuJoCo model from the D1 wire protocol.
class MujocoArmBridge
{
public:
  MujocoArmBridge(mjModel* model, mjData* data, const MujocoArmLayout& layout)
  : mj_model_(model), mj_data_(data), base_(layout.base), n_(layout.count)
  {
    cmd_ = std::make_unique<subscription::ArmCommand>();
    angle_ = std::make_unique<publisher::ServoAngle>();
    // The arm's driver publishes on rt/arm_Feedback, but Unitree's own
    // get_arm_joint_angle.cpp subscribes to arm_Feedback. Serve both.
    feedback_ = std::make_unique<publisher::ArmFeedback>(topic::kArmFeedback);
    feedback_alt_ = std::make_unique<publisher::ArmFeedback>(topic::kArmFeedbackAlt);

    for (int i = 0; i < n_; i++) {
      kp0_[i] = mj_model_->actuator_gainprm[mjNGAIN * (base_ + i) + 0];
      kv0_[i] = mj_model_->actuator_biasprm[mjNBIAS * (base_ + i) + 2];
      enabled_[i] = true;
      target_[i] = mj_data_->ctrl[base_ + i];
    }

    // Where each servo's measured angle lives in sensordata. Resolved by
    // walking actuator -> joint -> the jointpos sensor on that joint, so no
    // assumption is made about the order of the model's <sensor> block.
    sensor_adr_.assign(n_, -1);
    for (int i = 0; i < n_; i++) {
      const int a = base_ + i;
      if (mj_model_->actuator_trntype[a] != mjTRN_JOINT) continue;
      const int jnt = mj_model_->actuator_trnid[2 * a];
      for (int sid = 0; sid < mj_model_->nsensor; sid++) {
        if (mj_model_->sensor_type[sid] == mjSENS_JOINTPOS &&
            mj_model_->sensor_objid[sid] == jnt) {
          sensor_adr_[i] = mj_model_->sensor_adr[sid];
          break;
        }
      }
      if (sensor_adr_[i] < 0) {
        std::cerr << "[d1] warning: arm actuator "
                  << mj_id2name(mj_model_, mjOBJ_ACTUATOR, a)
                  << " has no <jointpos> sensor; its feedback will read 0"
                  << std::endl;
      }
    }
    apply();  // publish nothing yet, but leave the model in a defined state
  }

  void start()
  {
    const int period_us = static_cast<int>(1e6 / kControlRateHz);
    thread_ = std::make_shared<unitree::common::RecurrentThread>(
        "d1_arm_bridge", UT_CPU_ID_NONE, period_us, [this]() { this->run(); });
  }

  int base() const { return base_; }
  int count() const { return n_; }

private:
  /// Wire angle (degrees) -> the value mj_data->ctrl expects.
  double wireToCtrl(int i, double deg) const
  {
    return (i == kGripperIndex) ? gripperDegToMetres(deg) : deg * M_PI / 180.0;
  }

  /// Measured value -> wire angle (degrees).
  double ctrlToWire(int i, double v) const
  {
    return (i == kGripperIndex) ? gripperMetresToDeg(v) : v * 180.0 / M_PI;
  }

  // Gripper linkage: a crank of length A drives a coupler of length C whose far
  // end slides along the jaw axis, so servo angle maps to jaw opening through
  // the law of cosines. These constants still need calibrating against a real
  // D1-550 -- treat gripper commands as approximate until then.
  static constexpr double kCrankA = 0.020;
  static constexpr double kCouplerC = 0.030;
  static constexpr double kOffset = -0.0125;

  static double gripperDegToMetres(double deg)
  {
    const double u = kCrankA * std::sin(deg * M_PI / 180.0);
    const double disc = u * u + kCouplerC * kCouplerC - kCrankA * kCrankA;
    if (disc < 0.0) return 0.0;
    return u + std::sqrt(disc) + kOffset;
  }

  static double gripperMetresToDeg(double m)
  {
    const double b = m - kOffset;
    if (b <= 0.0) return 0.0;
    double s = (kCrankA * kCrankA + b * b - kCouplerC * kCouplerC) / (2.0 * kCrankA * b);
    s = std::max(-1.0, std::min(1.0, s));
    return std::asin(s) * 180.0 / M_PI;
  }

  /// Push targets and gain scaling into the model.
  void apply()
  {
    const double scale = powered_ ? stiffness_ : 0.0;
    for (int i = 0; i < n_; i++) {
      const bool live = enabled_[i] && powered_;
      const double gain = live ? kp0_[i] * scale : 0.0;
      mj_model_->actuator_gainprm[mjNGAIN * (base_ + i) + 0] = gain;
      mj_model_->actuator_biasprm[mjNBIAS * (base_ + i) + 1] = -gain;
      mj_model_->actuator_biasprm[mjNBIAS * (base_ + i) + 2] = live ? kv0_[i] * scale : 0.0;
      mj_data_->ctrl[base_ + i] = target_[i];
    }
  }

  void handle(const Command& c)
  {
    bool ok = true;
    switch (c.funcode) {
      case funcode::kJointAngle:
        if (c.id < n_) target_[c.id] = wireToCtrl(c.id, c.angle_deg);
        else ok = false;
        break;
      case funcode::kAllJointAngle:
        for (int i = 0; i < n_; i++) {
          if (c.has_angle[i]) target_[i] = wireToCtrl(i, c.angles[i]);
        }
        break;
      case funcode::kJointEnable:
        if (c.id < n_) enabled_[c.id] = (c.enable_mode != 0);
        else ok = false;
        break;
      case funcode::kAllJointEnable:
        for (int i = 0; i < n_; i++) enabled_[i] = (c.enable_mode > 0);
        // The guide gives 0..80000, but the SDK's own example sends {"mode":1}
        // to mean "enabled". Treat 1 as nominal gain rather than nearly zero.
        stiffness_ = (c.enable_mode == 1)
                         ? 1.0
                         : std::min(static_cast<double>(c.enable_mode) / kStiffnessMax, 1.0);
        break;
      case funcode::kPower:
        powered_ = (c.power != 0);
        break;
      case funcode::kZero:
        for (int i = 0; i < n_; i++) target_[i] = 0.0;
        break;
      default:
        ok = false;
        break;
    }
    if (ok) apply();
    feedback_->SendRecvAck(c.seq, 1);
    feedback_alt_->SendRecvAck(c.seq, 1);
    feedback_->SendExecAck(c.seq, ok ? 1 : 0);
    feedback_alt_->SendExecAck(c.seq, ok ? 1 : 0);
  }

  void run()
  {
    if (!mj_data_) return;

    if (cmd_->valid) {
      cmd_->valid = false;  // consume before handling
      handle(cmd_->command);
    }

    ServoAngles wire{};
    for (int i = 0; i < n_; i++) {
      const int adr = sensor_adr_[i];
      wire[i] = (adr >= 0) ? static_cast<float>(ctrlToWire(i, mj_data_->sensordata[adr])) : 0.f;
    }
    angle_->Send(wire);
    feedback_->SendAngles(wire);
    feedback_alt_->SendAngles(wire);

    if (++tick_ % 10 == 0) {  // status at ~1 Hz
      bool any = false;
      for (int i = 0; i < n_; i++) any = any || enabled_[i];
      feedback_->SendStatus(any ? 1 : 0, powered_ ? 1 : 0, 1);
      feedback_alt_->SendStatus(any ? 1 : 0, powered_ ? 1 : 0, 1);
    }
  }

  mjModel* mj_model_;
  mjData* mj_data_;
  int base_;
  int n_;

  std::vector<int> sensor_adr_;  ///< sensordata address per servo, -1 when absent

  double target_[kNumServo] = {0};
  double kp0_[kNumServo] = {0};
  double kv0_[kNumServo] = {0};
  bool enabled_[kNumServo] = {false};
  bool powered_ = true;
  double stiffness_ = 1.0;
  long tick_ = 0;

  std::unique_ptr<subscription::ArmCommand> cmd_;
  std::unique_ptr<publisher::ServoAngle> angle_;
  std::unique_ptr<publisher::ArmFeedback> feedback_;
  std::unique_ptr<publisher::ArmFeedback> feedback_alt_;
  unitree::common::RecurrentThreadPtr thread_;
};

/// Detect an arm in the model and, if there is one, start serving it.
///
/// Returns nullptr when the model carries no D1, so the call is safe to make
/// unconditionally. Requires ChannelFactory to be initialised already.
inline std::unique_ptr<MujocoArmBridge> StartMujocoArmBridge(mjModel* m, mjData* d)
{
  const MujocoArmLayout layout = MujocoArmLayout::Detect(m);
  if (!layout.valid()) return nullptr;
  auto bridge = std::make_unique<MujocoArmBridge>(m, d, layout);
  bridge->start();
  std::cout << "[d1] serving the arm on " << topic::kArmCommand << ": ctrl["
            << layout.base << ".." << layout.base + layout.count - 1 << "] at "
            << kControlRateHz << " Hz" << std::endl;
  return bridge;
}

}  // namespace d1
}  // namespace robot
}  // namespace unitree
