// Move one joint to an angle typed in at runtime.
//
// joint_angle_control.cpp carries its target inside the source, so trying a
// different joint or a different angle means an edit and a rebuild. This one
// reads the joint id and the angle (DEGREES) from the terminal instead, so one
// binary can walk through as many poses as you like:
//
//   ./joint_angle_control_interactive lo             # interactive, many moves
//   ./joint_angle_control_interactive lo 1 90        # one move, then exit
//   ./joint_angle_control_interactive lo 1 90 500    # ... executed over 500 ms
//
// It speaks the same funcode 1 command as the stock example -- through this
// package's wrapper rather than a hand-built JSON string -- so it drives the
// real arm and the MuJoCo bridge alike.

#include <unitree_arm/dds_wrapper/d1/d1.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

using namespace unitree::robot;

namespace
{

/// The publisher writes from its own thread; give it time to reach the wire
/// before returning to the prompt (or exiting).
constexpr int kSettleMs = 300;

void PrintUsage(const char* prog)
{
    std::cout
        << "Usage: " << prog << " <network_interface> [joint_id] [angle_deg] [delay_ms]\n"
        << "  With joint_id and angle_deg: send that one move and exit.\n"
        << "  Without them: read moves from the terminal, one per line.\n";
}

void PrintHelp()
{
    std::cout
        << "\nCommands (joint ids 0.." << d1::kNumServo - 1
        << ", J" << d1::kGripperIndex << " is the gripper):\n"
        << "  <id> <angle_deg> [delay_ms]   move one joint, e.g. \"1 90\" or \"1 90 500\"\n"
        << "  a                             print the latest reported joint angles\n"
        << "  z                             return every joint to zero (funcode 7)\n"
        << "  h                             this help\n"
        << "  q                             quit\n\n";
}

/// Reads the angles the arm last reported on current_servo_angle. Returns false
/// when nothing has arrived recently -- the arm may be off, or this may be a
/// send-only session against hardware that reports on a different interface.
bool ReadAngles(d1::subscription::ServoAngle& sub, d1::ServoAngles& out)
{
    if (sub.isTimeout()) return false;
    std::lock_guard<std::mutex> lock(sub.mutex_);
    out = sub.angles;
    return true;
}

void PrintAngles(d1::subscription::ServoAngle& sub)
{
    d1::ServoAngles angles{};
    if (!ReadAngles(sub, angles)) {
        std::cout << "  (no angle feedback on " << d1::topic::kServoAngle << ")" << std::endl;
        return;
    }
    std::cout << "  current:";
    for (int i = 0; i < d1::kNumServo; ++i) {
        std::cout << " J" << i << "=" << angles[i];
    }
    std::cout << " [deg]" << std::endl;
}

/// Rejects what the protocol cannot carry: an id outside the servo range, a
/// non-finite angle (EncodeJointAngle would refuse it anyway), or a delay that
/// does not fit the int16_t on the wire.
bool ValidateMove(long id, double angle_deg, long delay_ms)
{
    if (id < 0 || id >= d1::kNumServo) {
        std::cout << "[ERROR] joint id must be 0.." << d1::kNumServo - 1 << std::endl;
        return false;
    }
    if (!std::isfinite(angle_deg)) {
        std::cout << "[ERROR] angle must be a finite number of degrees." << std::endl;
        return false;
    }
    if (delay_ms < 0 || delay_ms > 32767) {
        std::cout << "[ERROR] delay_ms must be 0..32767." << std::endl;
        return false;
    }
    return true;
}

bool SendMove(d1::publisher::ArmCommand& cmd, long id, double angle_deg, long delay_ms)
{
    if (!ValidateMove(id, angle_deg, delay_ms)) return false;

    std::cout << "Sending J" << id << " -> " << angle_deg << " deg";
    if (delay_ms > 0) std::cout << " over " << delay_ms << " ms";
    std::cout << " ..." << std::endl;

    if (!cmd.SendJointAngle(static_cast<uint8_t>(id),
                            static_cast<float>(angle_deg),
                            static_cast<int16_t>(delay_ms))) {
        std::cout << "[ERROR] the command could not be encoded; nothing was sent." << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kSettleMs));
    return true;
}

/// Handles one line of input. Returns false when the user asked to quit.
bool HandleLine(const std::string& line,
                d1::publisher::ArmCommand& cmd,
                d1::subscription::ServoAngle& angle_sub)
{
    std::istringstream in(line);
    std::string first;
    if (!(in >> first)) return true;  // blank line

    if (first == "q" || first == "quit" || first == "exit") return false;
    if (first == "h" || first == "help" || first == "?") { PrintHelp(); return true; }
    if (first == "a" || first == "angles") { PrintAngles(angle_sub); return true; }
    if (first == "z" || first == "zero") {
        std::cout << "Sending zero ..." << std::endl;
        cmd.SendZero();
        std::this_thread::sleep_for(std::chrono::milliseconds(kSettleMs));
        PrintAngles(angle_sub);
        return true;
    }

    long id = 0;
    double angle_deg = 0.0;
    long delay_ms = 0;
    std::istringstream id_stream(first);
    if (!(id_stream >> id) || !id_stream.eof() || !(in >> angle_deg)) {
        std::cout << "[ERROR] expected \"<id> <angle_deg> [delay_ms]\"; type h for help."
                  << std::endl;
        return true;
    }
    if (in >> delay_ms) {
        // optional third field; anything past it is ignored
    } else {
        delay_ms = 0;
    }

    if (SendMove(cmd, id, angle_deg, delay_ms)) PrintAngles(angle_sub);
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    // Network interface check, as in every other example.
    if (argc < 2) {
        std::cout << "[ERROR] Please type in network interface name." << std::endl;
        PrintUsage(argv[0]);
        return -1;
    }
    std::string net_if = argv[1];
    std::cout << "Initializing DDS to the typed network interface (" << net_if << ") ..."
              << std::endl;

    // Network interface binding, Domain ID 0
    ChannelFactory::Instance()->Init(0, net_if);

    std::cout << "Generating publisher/subscriber channels ..." << std::endl;
    d1::publisher::ArmCommand cmd;
    d1::subscription::ServoAngle angle_sub;

    // One-shot mode: joint id and angle came in as arguments.
    if (argc >= 4) {
        long id = 0;
        double angle_deg = 0.0;
        long delay_ms = 0;
        std::istringstream args(std::string(argv[2]) + " " + argv[3] +
                                (argc >= 5 ? std::string(" ") + argv[4] : std::string()));
        if (!(args >> id >> angle_deg) || (argc >= 5 && !(args >> delay_ms))) {
            std::cout << "[ERROR] could not read joint id / angle / delay from the arguments."
                      << std::endl;
            PrintUsage(argv[0]);
            return -1;
        }
        if (!SendMove(cmd, id, angle_deg, delay_ms)) return -1;
        PrintAngles(angle_sub);
        std::cout << "Success: Program terminated normally." << std::endl;
        return 0;
    }

    if (argc == 3) {
        std::cout << "[ERROR] an angle must accompany the joint id." << std::endl;
        PrintUsage(argv[0]);
        return -1;
    }

    // Interactive mode.
    std::cout << "Interactive mode: type a joint id and an angle in degrees." << std::endl;
    PrintHelp();

    std::string line;
    while (true) {
        std::cout << "d1> " << std::flush;
        if (!std::getline(std::cin, line)) {  // EOF, e.g. Ctrl-D or a piped script
            std::cout << std::endl;
            break;
        }
        if (!HandleLine(line, cmd, angle_sub)) break;
    }

    std::cout << "Success: Program terminated normally." << std::endl;
    return 0;
}
