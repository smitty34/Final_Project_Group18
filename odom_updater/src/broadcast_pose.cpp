#include <functional>
#include <memory>
#include <sstream>
#include <string>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;


class BroadcastPose : public rclcpp::Node
{

public:
    BroadcastPose() : Node("odom_updater")
    {
        base_footprint_ = this->declare_parameter<std::string>("/robot_pose", "base_footprint");


        std::ostringstream stream;
        stream << "/robot1" << base_footprint_.c_str();
        std::string topic_name = stream.str();

    

    // Initialize the transform broadcaster

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/robot1/odom", 10,
                                                                            std::bind(&BroadcastPose::odom_callback, this, _1));

    }

private:


    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        rclcpp::Time now;

        geometry_msgs::msg::TransformStamped odom_tf;
        geometry_msgs::msg::TransformStamped base_foot_tf;

        odom_tf.header.stamp = now;
        base_foot_tf.header.stamp = now;


        base_foot_tf.transform.translation.x = msg->pose.pose.position.x;
        base_foot_tf.transform.translation.y = msg->pose.pose.position.y;
        base_foot_tf.transform.translation.z = 0.0;
        base_foot_tf.transform.rotation = msg->pose.pose.orientation;

        base_foot_tf.header.frame_id = "odom";
        base_foot_tf.child_frame_id = base_footprint_.c_str();



        // Odom to World Frame
        odom_tf.transform.translation.x = 0.0;
        odom_tf.transform.translation.x = 0.0;
        odom_tf.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0,0,0);


        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();


        odom_tf.header.frame_id = "world";
        odom_tf.child_frame_id = "odom";

        tf_broadcaster_->sendTransform(odom_tf);
 
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::string base_footprint_;

};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BroadcastPose>());
    rclcpp::shutdown();



    return 0;
}