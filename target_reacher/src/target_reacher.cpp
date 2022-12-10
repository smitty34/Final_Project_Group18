#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"



// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {

        m_bot_controller = bot_controller;

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


        m_bot_controller->set_goal(0,0);

        
        move_bot = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

        marker_found = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/waypoints", 10, std::bind(&TargetReacher::Arucocallback, this, std::placeholders::_1));
      
        image_detector = create_subscription<sensor_msgs::msg::Image>("/robot1/camera_rgb_frame", 10, std::bind(&TargetReacher::img_finder, this, std::placeholders::_1));

      
    }


    

private:




    void Arucocallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {

        visualization_msgs::msg::Marker artag;

        

        m_bot_controller->set_goal(x_0, y_0);

        while (goal_reached != true)
        {

            

            if (x_0 == 1.0 && y_0 == 1.0){

                artag.header.frame_id = "origin0";
                artag.pose.position.x = x_0;
                artag.pose.position.y = y_0;

                scan_n_search();

                m_bot_controller->set_goal(6.5, 5.0);

                goal_reached = true;

            }

            else if (x_1 == 1.0 && y_0 == -1.0){

                

                artag.header.frame_id = "origin1";
                artag.pose.position.x = x_0;
                artag.pose.position.y = y_0;

                scan_n_search();
                m_bot_controller->set_goal(6.5, 5.0);

                goal_reached = true;
                
            }


            else if (x_1 == -1.0 && y_0 == -1.0){


                artag.header.frame_id = "origin2";
                artag.pose.position.x = x_0;
                artag.pose.position.y = y_0;

                scan_n_search();
                m_bot_controller->set_goal(6.5, 5.0);

                goal_reached = true;
            }


            else if (x_1 == -1.0 && y_0 == 1.0){


                artag.header.frame_id = "origin3";
                artag.pose.position.x = x_0;
                artag.pose.position.y = y_0;

                scan_n_search();
                m_bot_controller->set_goal(6.5, 5.0);

                goal_reached = true;

            }

        }
            


    }
    
    void img_finder(sensor_msgs::msg::Image::ConstSharedPtr image_msg)
    {
        rclcpp::Time now;

        
        ros2_aruco_interfaces::msg::ArucoMarkers ar_tag;

        geometry_msgs::msg::TransformStamped trans_img;

        ar_tag.header.frame_id = image_msg->header.frame_id;
        
        
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            {
                cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            }
        }
        catch(cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        auto image = cv_ptr->image.clone();
        
        cv::Mat imageCopy;

        cv_ptr->image.copyTo(image);
        image.copyTo(imageCopy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(image, dictionary, corners, ids);
       

        int count = 0;
       

        for (unsigned int i = 0; i< ids.size(); i++) {
            RCLCPP_DEBUG(this->get_logger(), "Seen ARUCO marker number %i", ids[i]); 
            
            if(ids[i] == 0) {
                count++;

            }
        }

        if (count > 0){


            ar_tag.header.stamp = now;
            ar_tag.header.frame_id = "robot1/camera_rgb_optical_frame";

            trans_img.header.frame_id = "robot1/camera_rgb_frame";
            trans_img.child_frame_id = ar_tag.header.frame_id;
            trans_img.transform.translation.x = ar_tag.marker_ids.at(x_goal);
            trans_img.transform.translation.y = ar_tag.marker_ids.at(y_goal);

            tf_broadcaster_->sendTransform(trans_img);

           
            
        }

        


    }




    void scan_n_search()
    {

        geometry_msgs::msg::Twist msg;

        msg.linear.x = 0.1;
        msg.angular.y = -0.2;

        move_bot->publish(msg);


    }



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

    cv::Ptr<cv::aruco::Dictionary> dictionary;

    std::string marker_frame;
    std::string camera_frame;
    std::string reference_frame;

    int target_number = 14;

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr marker_found;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_detector;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_bot;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr final_goal;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};


