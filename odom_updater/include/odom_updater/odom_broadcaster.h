#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

/**
 * @brief Class to publish dynamic frame to connect /robot1/odom to /robot1/base_footprint
 * 
 */
class DynamicFrameBroadcaster : public rclcpp::Node
{
  /**
 * @brief Constructor to start the Node.
 * 
 */
public:
  DynamicFrameBroadcaster() 
  : Node("dynamic_tf2_frame_publisher")
  { 
    // Initialize transform broadcaster
    std::string topic_name = "/robot1/odom";
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    this->subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(topic_name, 10, std::bind(&DynamicFrameBroadcaster::broadcast_timer_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  /**
 * @brief Broadcast timer callback
 * 
 */
  void broadcast_timer_callback(const nav_msgs::msg::Odometry& msg);
};