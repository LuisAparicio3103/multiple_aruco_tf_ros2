
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

//#include <multiple_aruco_tf/CurrentPose.h>

using namespace std::chrono_literals;

class TfListenerNode : public rclcpp::Node
{
public:
    TfListenerNode() : Node("relative_pose")
    {
        // Create the TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>("relative_pose", 10);

        // Set up a timer to check for transforms
        timer_ = this->create_wall_timer(
            41.666666667ms, std::bind(&TfListenerNode::lookupTransform, this)
        );

        if (tf_end == "")
        {
          RCLCPP_ERROR(this->get_logger(), "Tf_end is not defined !!");
          //return -1;
        }
        else
          RCLCPP_INFO(rclcpp::get_logger("relative_pose_node"), "Will publish tf from %s to %s", tf_begin.c_str(), tf_end.c_str());
    }

private:
    void lookupTransform()
    {
        try
        {
            // Try to get the transform from 'frame_a' to 'frame_b'
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                tf_end, tf_begin, tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(), "Transform: [%f, %f, %f]",
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z);

            relative_pose.linear.x = t.transform.translation.x;
            relative_pose.linear.y = t.transform.translation.y;
            relative_pose.linear.z = t.transform.translation.z;

            tf2::Quaternion quat;
            tf2::fromMsg(t.transform.rotation, quat);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            relative_pose.angular.x = roll;
            relative_pose.angular.y = pitch;
            relative_pose.angular.z = yaw;
            
            publisher_->publish(relative_pose);  
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string tf_begin, tf_end;
    geometry_msgs::msg::Twist relative_pose;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfListenerNode>());

  //ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0), timerCallback);
  
  //pub = nh.advertise<geometry_msgs::Twist>("relative_pose", 10); 
  //pub = this->create_publisher<geometry_msgs::msg::Twist>("relative_pose", 10);
  
  //ros::ServiceServer service = nh.advertiseService("current_pose", srvCallback);
  
  //ROS_INFO("Relative pose is running!"); 
  //ros::spin();
  
  rclcpp::shutdown();
  return 0;
}