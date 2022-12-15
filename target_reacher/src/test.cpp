#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>



#include "target_reacher/target_reacher.h"



void TargetReacher::find_goal()
{
    m_bot_controller->set_goal(6.5,4.0);

}


void TargetReacher::bot_rotate()
{
    geometry_msgs::msg::Twist msg;
    msg.angular.z = 0.4;
    move_bot->publish(msg);
}


void TargetReacher::bot_stop()
{
    geometry_msgs::msg::Twist msg;
    msg.angular.z = 0;
    move_bot->publish(msg);
}


void TargetReacher::img_finder(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{

    go_tonext_area(msg);

    rclcpp::Time now;
    
    marker_id = msg->marker_ids.at(0);
    geometry_msgs::msg::TransformStamped bf_trans;

    if (marker_id > 0)
    {

        bf_trans.header.stamp = now;
        bf_trans.header.frame_id = "/robot1/camera/camera_rgb_optical_frame";
        bf_trans.child_frame_id = "/aruco_markers";

        bf_trans.transform.translation.x = msg->poses[0].position.x;
        bf_trans.transform.translation.y = msg->poses[0].position.y;
        bf_trans.transform.translation.z = msg->poses[0].position.z;

        bf_trans.transform.rotation.x = msg->poses[0].orientation.x;
        bf_trans.transform.rotation.y = msg->poses[0].orientation.y;
        bf_trans.transform.rotation.z = msg->poses[0].orientation.z;
        bf_trans.transform.rotation.w = msg->poses[0].orientation.w;

        tf_broadcaster_->sendTransform(bf_trans);


    }



}


// void TargetReacher::marker_listener(tf2_ros::Buffer& tf_buffer)
// {
//     geometry_msgs::msg::TransformStamped bf_trans;

    


//     try
//         {
//             bf_trans = tf_buffer.lookupTransform("world", "/aruco_markers", rclcpp::Time(0));

//             auto tf_x = bf_trans.transform.translation.x;
//             auto tf_y = bf_trans.transform.translation.y;
//             auto tf_z = bf_trans.transform.translation.z;

//             tf2::Quaternion q(bf_trans.transform.rotation.x, bf_trans.transform.rotation.y, bf_trans.transform.rotation.z, bf_trans.transform.rotation.w);

//             double roll;
//             double pitch;
//             double yaw;

//             tf2::Matrix3x3 m(q);

//             m.getRPY(roll, pitch, yaw);

//             RCLCPP_INFO(this->get_logger(), "Marker Reached!");

        
//         }

//         catch (tf2::TransformException& ex){

//             RCLCPP_WARN(this->get_logger(), "%s", ex.what());
//         }
// }




void TargetReacher::go_tonext_area(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    

    while (rclcpp::ok()){

        
        find_goal();



        if (msg->marker_ids.at(0) == 0){

            m_bot_controller->set_goal(x_0,y_0);

        }


        else if (msg->marker_ids.at(0) == 1){


            m_bot_controller->set_goal(x_1,y_1);

        }


        else if (msg->marker_ids.at(0) == 2){

            m_bot_controller->set_goal(x_2,y_2);

        }


        else if (msg->marker_ids.at(0) == 3){

            m_bot_controller->set_goal(x_3,y_3);

        }
    }

    
}

        



