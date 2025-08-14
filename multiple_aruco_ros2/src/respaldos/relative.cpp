
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

std::string tf_begin, tf_end;

//tf::StampedTransform transform;

geometry_msgs::msg::TransformStamped t;

geometry_msgs::msg::Twist relative_pose;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;

rclcpp::Node::SharedPtr node;

rclcpp::TimerBase::SharedPtr timer_;

std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};


void timer_callback(){
  
    try {
          t = tf_buffer_->lookupTransform(
            tf_end, tf_begin,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            node->get_logger(), "Could not transform %s to %s: %s",
            tf_end.c_str(), tf_begin.c_str(), ex.what());
          return;
        }
  	
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
		
	  pub->publish(relative_pose);  
    
  };

/*
void timerCallback(const ros::TimerEvent&)
{  
	tf::TransformListener listener;
	try{
		listener.waitForTransform(tf_begin, tf_end, ros::Time(), ros::Duration(5.0));
		listener.lookupTransform(tf_begin,  tf_end, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){return;} 
      
	  relative_pose.linear.x = transform.getOrigin().x();
    relative_pose.linear.y = transform.getOrigin().y();
    relative_pose.linear.z = transform.getOrigin().z();
    
    tf::Quaternion q = transform.getRotation(); 
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
		
	relative_pose.angular.x = roll;
	relative_pose.angular.y = pitch;
	relative_pose.angular.z = yaw;
		
	pub.publish(relative_pose);       
}

bool srvCallback(multiple_aruco_tf::CurrentPose::Request  &req,
         multiple_aruco_tf::CurrentPose::Response &res)
{
  res.pose = relative_pose;
  return true;
}
*/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("relative_pose_node");
  
  node->declare_parameter<std::string>("tf_begin", "marker_8");
  node->declare_parameter<std::string>("tf_end", "marker_7");

  node->get_parameter("tf_begin", tf_begin);
  node->get_parameter("tf_end", tf_end);


  if (tf_end == "")
  {
    RCLCPP_ERROR(node->get_logger(), "Tf_end is not defined !!");
    return -1;
  }
  else
    RCLCPP_INFO(rclcpp::get_logger("relative_pose_node"), "Will publish tf from %s to %s", tf_begin.c_str(), tf_end.c_str());

  //ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0), timerCallback);

  timer_ = node->create_wall_timer(41.666666667ms, timer_callback); //Calcular ms -> (1/24)*1000
  
  //pub = nh.advertise<geometry_msgs::Twist>("relative_pose", 10); 
  pub = node->create_publisher<geometry_msgs::msg::Twist>("relative_pose", 10);
  
        
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  //ros::ServiceServer service = nh.advertiseService("current_pose", srvCallback);
  
  //ROS_INFO("Relative pose is running!"); 
  //ros::spin();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}