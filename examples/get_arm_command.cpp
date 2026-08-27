// Command monitor: subscribes to the same topic the control executables publish to,
// so the outgoing joint commands can be inspected while no simulator/hardware is attached.

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree_arm/idl/ArmString_.hpp>

#define TOPIC "rt/arm_Command"

#include <iostream>
#include <unistd.h>

using namespace unitree::robot;
using namespace unitree::common;

void Handler(const void* msg)
{
    const unitree_arm::msg::dds_::ArmString_* pm = (const unitree_arm::msg::dds_::ArmString_*)msg;

    std::cout << "armCommand_data:" << pm->data_() << std::endl;
}

int main(int argc, char** argv)
{
    // Network interface check
    if (argc < 2) {
        std::cout << "[ERROR] Please type in network interface name." << std::endl;
        return -1;
    }
    std::string net_if = argv[1];
    std::cout << "Initializing DDS to the typed network interface (" << net_if << ") ..." << std::endl;

    // 2. Network interface binding, Domain ID 0
    ChannelFactory::Instance()->Init(0, net_if);

    std::cout << "Generating subscriber channels ..." << std::endl;
    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber(TOPIC);
    subscriber.InitChannel(Handler);

    std::cout << "Waiting for incoming packets ..." << std::endl;
    while (true)
    {
        sleep(10);
    }

    return 0;
}
