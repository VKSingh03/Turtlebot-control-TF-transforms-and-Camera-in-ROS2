#include "target_reacher.h"

void TargetReacher::goal_detect_callback(const std_msgs::msg::Bool& msg)
    {
      if(msg.data==true){
           geometry_msgs::msg::Twist velocity;
           velocity.angular.z=0.2;
           this->velocity_publisher->publish(velocity); 
        }
    }

void TargetReacher::aruco_detect_callback(const ros2_aruco_interfaces::msg::ArucoMarkers& arc)
{   
    if(!arc.marker_ids[0]){
        //Variables to form the final destination aruco marker id from detected aruco marker 
        std::string final_goalx = "final_destination.aruco_" + std::to_string(arc.marker_ids[0])+".x"; 
        std::string final_goaly = "final_destination.aruco_" + std::to_string(arc.marker_ids[0])+".y";
        double final_goal_x=this->get_parameter(final_goalx).get_parameter_value().get<double>(); 
        double final_goal_y=this->get_parameter(final_goaly).get_parameter_value().get<double>(); 
        std::string final_goal_frame =this->get_parameter("final_destination.frame_id").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "Aruco detect callback");
        // Call the publisher to enable proper publication of the frame. Suggested by Professor. 
        for (int i=0; i<10; i++){
            TargetReacher::timer_callback_broadcaster(final_goal_x, final_goal_y, final_goal_frame); //
        }
        // Call to subscriber to read the new frame publisherd
        TargetReacher::new_frame_subscriber();
        // Call to bot controller to go the final goal position.
        m_bot_controller->set_goal(x_goal2,y_goal2);
    }
}

void TargetReacher::timer_callback_broadcaster(double final_goal_x, double final_goal_y, std::string final_goal_frame)
{    
    RCLCPP_INFO(this->get_logger(), "Broadcaster callback");
    geometry_msgs::msg::TransformStamped t;
    // Read message content and assign it to corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = final_goal_frame; //
    t.child_frame_id = "final_destination";
    // Position
    t.transform.translation.x = final_goal_x;
    t.transform.translation.y = final_goal_y;
    t.transform.translation.z = 0.0;
    // Quaternion
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    // Send the transformation
    final_goal_frame_broadcast->sendTransform(t);
}

void TargetReacher::new_frame_subscriber() 
{   
    RCLCPP_INFO(this->get_logger(), "Listener callback");
    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between final_destination and robot1/odom frames
    try
    {
        t = m_tf_buffer->lookupTransform("final_destination", "robot1/odom", tf2::TimePointZero);
        x_goal2 = t.transform.translation.x;
        y_goal2 = t.transform.translation.y;
        
    }
    catch (const tf2::TransformException &ex)
    {
        // RCLCPP_INFO(
        //     this->get_logger(), "Could not transform %s to %s: %s",
        //     "final_destination", "/robot1/odom", ex.what());
        // return;
    }
    // Puclish final goal on screen. 
    RCLCPP_INFO(
        this->get_logger(), "Position of object in odom: [%f, %f]", x_goal2, y_goal2);
}