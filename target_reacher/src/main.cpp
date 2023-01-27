#include <rclcpp/rclcpp.hpp>
#include "target_reacher.h"
#include "bot_controller/bot_controller.h"
// main function
int main(int argc, char *argv[])
{   // init
    rclcpp::init(argc, argv);
    // create pointer to bot controller
    auto bot_controller = std::make_shared<BotController>("bot_controller_robot", "robot1");
    // Multithreaded executr
    rclcpp::executors::MultiThreadedExecutor exec;
    // Creating node.
    auto node = std::make_shared<TargetReacher>(bot_controller);
    exec.add_node(node);
    exec.add_node(bot_controller);
    // Spin node
    exec.spin();
    // Shutdown
    rclcpp::shutdown();
}