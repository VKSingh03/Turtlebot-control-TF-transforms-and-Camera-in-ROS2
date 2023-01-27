#include "odom_broadcaster.h"

void DynamicFrameBroadcaster::broadcast_timer_callback(const nav_msgs::msg::Odometry& msg)
{
    geometry_msgs::msg::TransformStamped t;
    // Read message content and assign it to corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot1/odom";
    t.child_frame_id = "robot1/base_footprint";
    // Position
    t.transform.translation.x = msg.pose.pose.position.x;
    t.transform.translation.y = msg.pose.pose.position.y;
    t.transform.translation.z = msg.pose.pose.position.z;
    // Quaternion
    t.transform.rotation.x = msg.pose.pose.orientation.x;
    t.transform.rotation.y = msg.pose.pose.orientation.y;
    t.transform.rotation.z = msg.pose.pose.orientation.z;
    t.transform.rotation.w = msg.pose.pose.orientation.w;
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
}

