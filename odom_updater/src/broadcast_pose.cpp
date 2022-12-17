#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <nav_msgs/msg/odometry.hpp>

// using std::placeholders::_1;


class BroadcastPose : public rclcpp::Node
{

public:
    BroadcastPose() : Node("odom_updater")
    {
   
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/robot1/odom", 10,
                                                                            std::bind(&BroadcastPose::odom_callback, this, std::placeholders::_1));




    }


protected:

    void robot_footprint(geometry_msgs::msg::TransformStamped t_msg)
    {
        tf_broadcaster_->sendTransform(t_msg);
    }

    void final_destination(geometry_msgs::msg::TransformStamped t_msg)
    {
        tf_broadcaster_->sendTransform(t_msg);
    }


    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
       

        geometry_msgs::msg::TransformStamped t_msg;

        t_msg.header.stamp = this->get_clock()->now();
        t_msg.header.frame_id = "/robot1/odom";
        t_msg.child_frame_id = "/robot1/base_footprint";

        t_msg.transform.translation.x = msg->pose.pose.position.x;
        t_msg.transform.translation.y = msg->pose.pose.position.y;
        t_msg.transform.translation.z = msg->pose.pose.position.z;


        t_msg.transform.rotation.x = msg->pose.pose.orientation.x;
        t_msg.transform.rotation.y = msg->pose.pose.orientation.y;
        t_msg.transform.rotation.z = msg->pose.pose.orientation.z;
        t_msg.transform.rotation.w = msg->pose.pose.orientation.w;


        BroadcastPose::robot_footprint(t_msg);

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "origin1";
        t.child_frame_id = "final_destination";

        t.transform.translation.x = 6.0;
        t.transform.translation.y = 4.5;
        t.transform.translation.z = 0;


        t.transform.rotation.x = 0;
        t.transform.rotation.y = 0;
        t.transform.rotation.z = 0;
        t.transform.rotation.w = 1;

        BroadcastPose::robot_footprint(t);


    }



private:

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    

};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<BroadcastPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();


}
