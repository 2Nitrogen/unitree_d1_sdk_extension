#include <unitree_arm/dds_wrapper/d1/d1_protocol.h>

#include <unitree/common/json/json.hpp>
#include <unitree/common/json/jsonize.hpp>

#include <cmath>
#include <iomanip>
#include <sstream>

namespace unitree
{
namespace robot
{
namespace d1
{
namespace
{

using unitree::common::Any;
using unitree::common::JsonMap;

std::ostringstream MakeStream()
{
  std::ostringstream os;
  os << std::defaultfloat << std::setprecision(9);
  return os;
}

bool AllFinite(const ServoAngles& a)
{
  for (float v : a) {
    if (!std::isfinite(v)) return false;
  }
  return true;
}

/// The envelope every message shares
std::string Envelope(int32_t seq, int address, int funcode, const std::string& data)
{
  std::ostringstream os = MakeStream();
  os << "{\"seq\":" << seq << ",\"address\":" << address << ",\"funcode\":" << funcode;
  if (!data.empty()) {
    os << ",\"data\":" << data;
  }
  os << "}";
  return os.str();
}

std::string AnglesObject(const ServoAngles& a, const std::string& prefix)
{
  std::ostringstream os = MakeStream();
  os << "{";
  for (int i = 0; i < kNumServo; ++i) {
    if (i) os << ",";
    os << "\"" << prefix << i << "\":" << a[i];
  }
  os << "}";
  return os.str();
}

/// unitree::common::Any holds numbers as int64 or double depending on how the text was written, so every numeric read goes through these.
bool AsDouble(const JsonMap& m, const char* key, double& out)
{
  auto it = m.find(key);
  if (it == m.end()) return false;
  const Any& a = it->second;
  if (a.GetTypeInfo() == typeid(double)) { out = unitree::common::AnyCast<double>(a); return true; }
  if (a.GetTypeInfo() == typeid(int64_t)) { out = double(unitree::common::AnyCast<int64_t>(a)); return true; }
  if (a.GetTypeInfo() == typeid(int32_t)) { out = double(unitree::common::AnyCast<int32_t>(a)); return true; }
  if (a.GetTypeInfo() == typeid(bool)) { out = unitree::common::AnyCast<bool>(a) ? 1.0 : 0.0; return true; }
  return false;
}

bool AsInt(const JsonMap& m, const char* key, int& out)
{
  double d = 0.0;
  if (!AsDouble(m, key, d)) return false;
  out = int(d);
  return true;
}

bool ParseEnvelope(const std::string& json, JsonMap& root, int32_t& seq,
                   int& address, int& funcode, JsonMap& data, bool& has_data)
{
  try {
    Any a = unitree::common::FromJsonString(json);
    if (!unitree::common::IsJsonMap(a)) return false;
    root = unitree::common::AnyCast<JsonMap>(a);
  } catch (...) {
    return false;
  }
  int s = 0;
  if (!AsInt(root, "seq", s)) return false;
  seq = int32_t(s);
  if (!AsInt(root, "address", address)) return false;
  if (!AsInt(root, "funcode", funcode)) return false;

  has_data = false;
  auto it = root.find("data");
  if (it != root.end() && unitree::common::IsJsonMap(it->second)) {
    try {
      data = unitree::common::AnyCast<JsonMap>(it->second);
      has_data = true;
    } catch (...) {
      return false;
    }
  }
  return true;
}

}  // namespace

// commands
std::string EncodeJointAngle(int32_t seq, uint8_t id, float angle_deg, int16_t delay_ms)
{
  if (!std::isfinite(angle_deg)) return std::string();
  std::ostringstream d = MakeStream();
  d << "{\"id\":" << int(id) << ",\"angle\":" << angle_deg
    << ",\"delay_ms\":" << delay_ms << "}";
  return Envelope(seq, address::kCommand, funcode::kJointAngle, d.str());
}

std::string EncodeAllJointAngle(int32_t seq, const ServoAngles& angle_deg, int mode)
{
  if (!AllFinite(angle_deg)) return std::string();
  std::ostringstream d = MakeStream();
  d << "{\"mode\":" << mode;
  for (int i = 0; i < kNumServo; ++i) d << ",\"angle" << i << "\":" << angle_deg[i];
  d << "}";
  return Envelope(seq, address::kCommand, funcode::kAllJointAngle, d.str());
}

std::string EncodeJointEnable(int32_t seq, uint8_t id, int mode)
{
  std::ostringstream d = MakeStream();
  d << "{\"id\":" << int(id) << ",\"mode\":" << mode << "}";
  return Envelope(seq, address::kCommand, funcode::kJointEnable, d.str());
}

std::string EncodeAllJointEnable(int32_t seq, int mode)
{
  std::ostringstream d = MakeStream();
  d << "{\"mode\":" << mode << "}";
  return Envelope(seq, address::kCommand, funcode::kAllJointEnable, d.str());
}

std::string EncodePower(int32_t seq, int power)
{
  std::ostringstream d = MakeStream();
  d << "{\"power\":" << power << "}";
  return Envelope(seq, address::kCommand, funcode::kPower, d.str());
}

std::string EncodeZero(int32_t seq)
{
  return Envelope(seq, address::kCommand, funcode::kZero, "");
}

// feedback
std::string EncodeAngleFeedback(const ServoAngles& angle_deg, int32_t seq)
{
  if (!AllFinite(angle_deg)) return std::string();
  return Envelope(seq, address::kFeedback, funcode::kAngleFeedback,
                  AnglesObject(angle_deg, "angle"));
}

std::string EncodeStatusFeedback(int enable_status, int power_status,
                                 int error_status, int32_t seq)
{
  std::ostringstream d = MakeStream();
  d << "{\"enable_status\":" << enable_status
    << ",\"power_status\":" << power_status
    << ",\"error_status\":" << error_status << "}";
  return Envelope(seq, address::kFeedback, funcode::kStatusFeedback, d.str());
}

std::string EncodeMotorStatus(const std::array<int, kNumServo>& status, int32_t seq)
{
  std::ostringstream d = MakeStream();
  d << "{";
  for (int i = 0; i < kNumServo; ++i) {
    if (i) d << ",";
    d << "\"motor" << i << "_status\":" << status[i];
  }
  d << "}";
  return Envelope(seq, address::kFeedback, funcode::kMotorStatus, d.str());
}

std::string EncodeRecvAck(int32_t seq, int recv_status)
{
  std::ostringstream d = MakeStream();
  d << "{\"recv_status\":" << recv_status << "}";
  return Envelope(seq, address::kAck, funcode::kRecvAck, d.str());
}

std::string EncodeExecAck(int32_t seq, int exec_status)
{
  std::ostringstream d = MakeStream();
  d << "{\"exec_status\":" << exec_status << "}";
  return Envelope(seq, address::kAck, funcode::kExecAck, d.str());
}

// parsing
bool DecodeCommand(const std::string& json, Command& out)
{
  JsonMap root, data;
  int32_t seq = 0;
  int addr = 0, fc = 0;
  bool has_data = false;
  if (!ParseEnvelope(json, root, seq, addr, fc, data, has_data)) return false;
  if (addr != address::kCommand) return false;

  Command c;
  c.seq = seq;
  c.funcode = fc;
  switch (fc) {
    case funcode::kJointAngle: {
      int id = 0;
      double angle = 0.0;
      if (!AsInt(data, "id", id) || !AsDouble(data, "angle", angle)) return false;
      c.id = uint8_t(id);
      c.angle_deg = float(angle);
      int delay = 0;
      if (AsInt(data, "delay_ms", delay)) c.delay_ms = int16_t(delay);
      break;
    }
    case funcode::kAllJointAngle: {
      AsInt(data, "mode", c.mode);
      for (int i = 0; i < kNumServo; ++i) {
        const std::string key = "angle" + std::to_string(i);
        double v = 0.0;
        if (AsDouble(data, key.c_str(), v)) {
          c.angles[i] = float(v);
          c.has_angle[i] = true;
        }
      }
      break;
    }
    case funcode::kJointEnable: {
      int id = 0;
      if (!AsInt(data, "id", id) || !AsInt(data, "mode", c.enable_mode)) return false;
      c.id = uint8_t(id);
      break;
    }
    case funcode::kAllJointEnable:
      if (!AsInt(data, "mode", c.enable_mode)) return false;
      break;
    case funcode::kPower:
      if (!AsInt(data, "power", c.power)) return false;
      break;
    case funcode::kZero:
      break;  // carries no data
    default:
      return false;  // outside the documented function set
  }
  out = c;
  return true;
}

bool DecodeFeedback(const std::string& json, Feedback& out)
{
  JsonMap root, data;
  int32_t seq = 0;
  int addr = 0, fc = 0;
  bool has_data = false;
  if (!ParseEnvelope(json, root, seq, addr, fc, data, has_data)) return false;
  if (addr != address::kFeedback && addr != address::kAck) return false;

  Feedback f;
  f.seq = seq;
  f.address = addr;
  f.funcode = fc;
  if (addr == address::kFeedback) {
    if (fc == funcode::kAngleFeedback) {
      for (int i = 0; i < kNumServo; ++i) {
        double v = 0.0;
        if (AsDouble(data, ("angle" + std::to_string(i)).c_str(), v)) f.angles[i] = float(v);
      }
      f.has_angles = true;
    } else if (fc == funcode::kStatusFeedback) {
      AsInt(data, "enable_status", f.enable_status);
      AsInt(data, "power_status", f.power_status);
      AsInt(data, "error_status", f.error_status);
      f.has_status = true;
    } else if (fc == funcode::kMotorStatus) {
      for (int i = 0; i < kNumServo; ++i) {
        AsInt(data, ("motor" + std::to_string(i) + "_status").c_str(), f.motor_status[i]);
      }
      f.has_motor_status = true;
    } else {
      return false;
    }
  } else {
    if (fc == funcode::kRecvAck) {
      AsInt(data, "recv_status", f.recv_status);
    } else if (fc == funcode::kExecAck) {
      AsInt(data, "exec_status", f.exec_status);
    } else {
      return false;
    }
    f.has_ack = true;
  }
  out = f;
  return true;
}

}  // namespace d1
}  // namespace robot
}  // namespace unitree
