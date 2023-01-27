#include <rclcpp/rclcpp.hpp>
#include "odom_broadcaster.h"


int main(int argc, char *argv[]){
    // init
    rclcpp::init (argc, argv);
    //create the node. 
    auto node = std::make_shared<DynamicFrameBroadcaster>();
    //spin the node
    rclcpp::spin(node);
    //shutdown
    rclcpp::shutdown();
    return 0;

}
