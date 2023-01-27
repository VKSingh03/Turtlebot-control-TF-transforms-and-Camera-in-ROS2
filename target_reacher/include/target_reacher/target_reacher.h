#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <ostream>
#include <sstream>

/**
 * @file target_reacher.h
 * @author vsingh03 (vsingh03@umd.edu)
 * @author ssarjun (ssarjun@umd.edu)
 * @author rpakhala (rpakhala@umd.edu)
 * @brief Class to take the turtlebot from initial pose to final goal completing all tasks.
 * @version 0.1
 * @date 2022-12-15
 *
 */
class TargetReacher : public rclcpp::Node
{
    /**
     * @brief TargetReacher Constructor. 
     * Declares the variables
     * Starts all the nodes
     * 
     */
    public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {
        // Declaring the parameters 
        m_bot_controller = bot_controller;
        auto aruco_target_x = this->declare_parameter<double>("aruco_target.x");
        auto aruco_target_y = this->declare_parameter<double>("aruco_target.y");
        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y");
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");
        this->declare_parameter<std::string>("final_destination.frame_id");

        // Setting Goal 1 location.
        m_bot_controller->set_goal(aruco_target_x, aruco_target_y); 

        // Subscriber to topic /goal_reached. 
        this->goal_detect_subscriber = this->create_subscription<std_msgs::msg::Bool>("/goal_reached", 10, std::bind(&TargetReacher::goal_detect_callback, this, std::placeholders::_1));
        
        // Publisher for bot rotation to find aruco marker
        this->velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        // Subscriber to receive Aruco marker detection message. 
        this->aruco_detection_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&TargetReacher::aruco_detect_callback, this, std::placeholders::_1));

        // Final_goal frame broadcaster 
        final_goal_frame_broadcast = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // Final_goal frame subscriber
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    }

private:
    // attributes
    /**
   * @brief Pointer to access bot controller class.
   * 
   */
    std::shared_ptr<BotController> m_bot_controller;
    /**
   * @brief To store final goal positions.
   * 
   */
    double x_goal2, y_goal2;
    /**
   * @brief To detect goal reached status.
   * 
   */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_detect_subscriber;
    /**
   * @brief To publisher velocity command to robot.
   * 
   */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    /**
   * @brief To access aruco marker topic message.
   * 
   */
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_detection_subscriber;
    /**
   * @brief To Broadcast frame from Origin_i to final_destination.
   * 
   */
    std::shared_ptr<tf2_ros::TransformBroadcaster> final_goal_frame_broadcast;
    rclcpp::TimerBase::SharedPtr m_timer;
    /**
   * @brief Receiving final goal in robot1/odom.
   * 
   */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr final_goal_subscription;
    /**
   * @brief Receives final goal location.
   * 
   */
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    
    /**
   * @brief Goal detect callback function
   * 
   * @param std::pair<int,int> cell_no : Takes the current position of robot
   * @param int dir : Takes the current direction of robot 
   */
    void goal_detect_callback(const std_msgs::msg::Bool& msg);

    /**
   * @brief Aruco detect callback function. Subscribes to /aruco_marker topic to check detection of aruco marker.
   * @param const ros2_aruco_interfaces::msg::ArucoMarkers&
   */
    void aruco_detect_callback(const ros2_aruco_interfaces::msg::ArucoMarkers& arc);

    /**
   * @brief Frame Broadcaster Callback: Broadcasts final_goal_frame as final_destination frame.
   * @param std::string final_goal_frame : The final goal frame based on detected aruco marker
   * @param double final_goal_x : X position of final goal
   * @param double final_goal_y : y position of final goal
   * 
   */
    void timer_callback_broadcaster(double final_goal_x, double final_goal_y, std::string final_goal_frame);

    /**
   * @brief Transform lisnter to grab final goal position
   * 
   */
    void new_frame_subscriber();

};