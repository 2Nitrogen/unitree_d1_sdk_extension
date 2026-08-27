// #include <unitree/robot/channel/channel_publisher.hpp>
// #include <unitree/common/time/time_tool.hpp>
// #include <unitree_arm/idl/ArmString_.hpp>

// #define TOPIC "rt/arm_Command"

// using namespace unitree::robot;
// using namespace unitree::common;

// int main()
// {
//     ChannelFactory::Instance()->Init(0);
//     ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
//     publisher.InitChannel();

//     unitree_arm::msg::dds_::ArmString_ msg{};
//     msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
//     publisher.Write(msg);

//     return 0;
// }

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree_arm/idl/ArmString_.hpp>

#define TOPIC "rt/arm_Command"

#include <iostream>
#include <unistd.h>

using namespace unitree::robot;
using namespace unitree::common;

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

    std::cout << "Generating publisher channels ..." << std::endl;
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    std::cout << "Creating message and sending packet ..." << std::endl;
    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
    publisher.Write(msg);

    sleep(1);  // Make sure DDS socket buffer is flushed before process terminates

    std::cout << "Success: Program terminated normally." << std::endl;
    return 0;
}
