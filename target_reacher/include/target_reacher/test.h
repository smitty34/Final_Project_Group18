#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

// ROS CvBridge



// vector
#include <vector>



// timer
class TargetReacher : public rclcpp::Node
{

public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {

        m_bot_controller = bot_controller;

        m_bot_controller->set_goal(6.0,4.5);


        x_0 = this->declare_parameter<double>("final_destination.aruco_0.x");
        y_0 = this->declare_parameter<double>("final_destination.aruco_0.y");

        x_1 = this->declare_parameter<double>("final_destination.aruco_1.x");
        y_1 = this->declare_parameter<double>("final_destination.aruco_1.y");

        x_2 = this->declare_parameter<double>("final_destination.aruco_2.x");
        y_2 = this->declare_parameter<double>("final_destination.aruco_2.y");

        x_3 = this->declare_parameter<double>("final_destination.aruco_3.x");
        y_3 = this->declare_parameter<double>("final_destination.aruco_3.y");

        x_goal = this->declare_parameter<double>("aruco_target.x");
        y_goal = this->declare_parameter<double>("aruco_target.y");


        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

       
        move_bot = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        marker_found = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&TargetReacher::img_finder, this, std::placeholders::_1));

        // marker_found = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&TargetReacher::Arucocallback, this, std::placeholders::_1));
      
        // image_detector = create_subscription<sensor_msgs::msg::Image>("/robot1/camera_rgb_frame", 10, std::bind(&TargetReacher::img_finder, this, std::placeholders::_1));



    }

    void find_goal();

    // void marker_listener(tf2_ros::Buffer& tf_buffer);

    void bot_rotate();

    void bot_stop();

    void go_tonext_area(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    void img_finder(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);



private:

        // attributes
    std::shared_ptr<BotController> m_bot_controller;
    double x_0 = 0;
    double y_0 = 0;
    double x_1 = 0;
    double y_1 = 0;
    double x_2 = 0;
    double y_2 = 0;
    double x_3 = 0;
    double y_3 = 0;
    double x_goal = 0;
    double y_goal = 0;
    bool found_tag = false;
    bool goal_reached = false;

    // cv::Ptr<cv::aruco::Dictionary> dictionary;

    std::string marker_frame;
    std::string camera_frame;
    std::string reference_frame;
    int marker_id = 5;


    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr marker_found;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_bot;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr final_goal;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    bool marker_id_found = false;



};
