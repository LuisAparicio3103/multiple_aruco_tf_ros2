#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

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
    TfListenerNode() : Node("relative_pose_node")
    {

        this->declare_parameter<std::string>("source_frame", "marker_7");
        this->declare_parameter<std::string>("target_frame", "marker_8");

        this->get_parameter("source_frame", source_frame);
        this->get_parameter("target_frame", target_frame);
        if (target_frame == "")
        {
          RCLCPP_ERROR(this->get_logger(), "target_frame is not defined !!");
          //return -1;
        }
        else
          //RCLCPP_INFO(rclcpp::get_logger("relative_pose_node"), "Will publish tf from %s to %s", source_frame.c_str(), target_frame.c_str());
        // Set up a timer to check for transforms

        // Create the TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>("relative_pose/" + source_frame + "_to_" + target_frame, 10);

        auto timer_callback=
        [this]() -> void{
            try
            {
                // Try to get the transform from 'frame_a' to 'frame_b'
                geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                //RCLCPP_INFO(rclcpp::get_logger("relative_pose_node"), "Will publish tf from %s to %s", source_frame.c_str(), target_frame.c_str());
    
               /* RCLCPP_INFO(this->get_logger(), "Transform: [%f, %f, %f]",
                            t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z);
    */
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
                //RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            }
        };
        
        timer_ = this->create_wall_timer(41.666666667ms, timer_callback);


    }
private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string source_frame;
    std::string target_frame;
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