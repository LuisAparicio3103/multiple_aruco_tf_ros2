#include <functional>
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include "rclcpp/rclcpp.hpp"

#include "aruco/aruco.h"
#include <aruco/cvdrawingutils.h>
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.hpp"


#include "sensor_msgs/image_encodings.hpp"
//#include "aruco_ros/aruco_ros_utils.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "turtlesim/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "tf2/transform_datatypes.h"

#include "tf2_ros/static_transform_broadcaster.h"
#include "opencv4/opencv2/calib3d.hpp"
#include "tf2/LinearMath/Transform.h"


/*

#include <dynamic_reconfigure/server.h> 
*/
//#include <aruco_ros/ArucoThresholdConfig.h>


cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages;
aruco::MarkerDetector mDetector; 
std::vector<aruco::Marker> markers;
//ros::Subscriber cam_info_sub; 
rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
bool cam_info_received;
//rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher");
rclcpp::Node::SharedPtr node;
image_transport::Publisher image_pub;
std::string parent_name;
std::string child_name; 
std::string dictionary_type;
double marker_size;
std::unique_ptr<tf2_ros::TransformBroadcaster> br;



void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{

  br = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
  
  if (cam_info_received)
  {

    rclcpp::Time curr_stamp = msg->header.stamp;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      markers.clear();
      mDetector.detect(inImage, markers, camParam, marker_size, false);
      
      for (unsigned int i = 0; i < markers.size(); ++i)
      {
        //--------------------------------aruco_ros::arucoMarker2Tf----------------------------
        
        cv::Mat rot(3, 3, CV_64FC1);
        cv::Mat Rvec64;
        markers[i].Rvec.convertTo(Rvec64, CV_64FC1);
        cv::Rodrigues(Rvec64, rot);
        cv::Mat tran64;
        markers[i].Tvec.convertTo(tran64, CV_64FC1);

        tf2::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
          rot.at<double>(1, 0),
          rot.at<double>(1, 1), rot.at<double>(1, 2), rot.at<double>(2, 0), rot.at<double>(2, 1),
          rot.at<double>(2, 2));

        tf2::Vector3 tf_orig(tran64.at<double>(0, 0), tran64.at<double>(1, 0), tran64.at<double>(2, 0));

        tf2::Transform transform = tf2::Transform(tf_rot, tf_orig);

        geometry_msgs::msg::Transform geometry_transform;

        geometry_transform.translation.x = transform.getOrigin().x();
        geometry_transform.translation.y = transform.getOrigin().y();
        geometry_transform.translation.z = transform.getOrigin().z();

        // Convertir la rotación de tf2::Quaternion a geometry_msgs::msg::Quaternion
        geometry_transform.rotation.x = transform.getRotation().x();
        geometry_transform.rotation.y = transform.getRotation().y();
        geometry_transform.rotation.z = transform.getRotation().z();
        geometry_transform.rotation.w = transform.getRotation().w();


        //--------------------------------------------------------------------------------------------

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = curr_stamp;
        t.header.frame_id = parent_name;
        t.child_frame_id = child_name + std::to_string(markers[i].id);
        t.transform = geometry_transform;
        br->sendTransform(t); 
        aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
        
      }

      if (image_pub.getNumSubscribers() > 0)
      {
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
        //RCLCPP_INFO(rclcpp::get_logger("image_publisher"), "¡Hola! Este es un mensaje de información.");
      }

      //ROS_DEBUG("runtime: %f ms", 1000 * (cv::getTickCount() - ticksBefore) / cv::getTickFrequency());
    }
    catch (cv_bridge::Exception& e)
    {
      //ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
  
}

void cam_info_callback(const sensor_msgs::msg::CameraInfo &cam_info)
{ 
  
  //camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

  //-----------------------rosCameraInfo2ArucoCamParams-------------------------
  if (cam_info_received == false){
    cv::Mat cameraMatrix(3, 4, CV_64FC1, 0.0);
    cv::Mat distorsionCoeff(4, 1, CV_64FC1);
    cv::Size size(cam_info.width, cam_info.height);

    if (useRectifiedImages) {
      cameraMatrix.setTo(0);
      cameraMatrix.at<double>(0, 0) = cam_info.p[0];
      cameraMatrix.at<double>(0, 1) = cam_info.p[1];
      cameraMatrix.at<double>(0, 2) = cam_info.p[2];
      cameraMatrix.at<double>(0, 3) = cam_info.p[3];
      cameraMatrix.at<double>(1, 0) = cam_info.p[4];
      cameraMatrix.at<double>(1, 1) = cam_info.p[5];
      cameraMatrix.at<double>(1, 2) = cam_info.p[6];
      cameraMatrix.at<double>(1, 3) = cam_info.p[7];
      cameraMatrix.at<double>(2, 0) = cam_info.p[8];
      cameraMatrix.at<double>(2, 1) = cam_info.p[9];
      cameraMatrix.at<double>(2, 2) = cam_info.p[10];
      cameraMatrix.at<double>(2, 3) = cam_info.p[11];

      for (int i = 0; i < 4; ++i) {
        distorsionCoeff.at<double>(i, 0) = 0;
      }
    } else {
      cv::Mat cameraMatrixFromK(3, 3, CV_64FC1, 0.0);
      for (int i = 0; i < 9; ++i) {
        cameraMatrixFromK.at<double>(i % 3, i - (i % 3) * 3) = cam_info.k[i];
      }
      cameraMatrixFromK.copyTo(cameraMatrix(cv::Rect(0, 0, 3, 3)));


      if (cam_info.d.size() == 4) {
        for (int i = 0; i < 4; ++i) {
          distorsionCoeff.at<double>(i, 0) = cam_info.d[i];
        }
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "image_publisher"), "length of camera_info D vector is not 4, assuming zero distortion...");
        for (int i = 0; i < 4; ++i) {
          distorsionCoeff.at<double>(i, 0) = 0;
        }
      }
    }

    camParam = aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);

  //----------------------------------------------------------------------------------------------------
  }
  cam_info_received = true;
  //cam_info_sub.shutdown();
  
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("image_publisher");
  image_transport::ImageTransport it(node);

  node->declare_parameter<double>("marker_size", 0.515);
  node->declare_parameter<std::string>("parent_name", "camera");
  node->declare_parameter<std::string>("child_name", "marker_");
  node->declare_parameter<std::string>("dictionary_type", "ARUCO_MIP_36h12");
  node->declare_parameter<bool>("image_is_rectified", true);

  node->get_parameter("marker_size", marker_size);
  node->get_parameter("parent_name", parent_name);
  node->get_parameter("child_name", child_name);
  node->get_parameter("dictionary_type", dictionary_type);
  node->get_parameter("image_is_rectified", useRectifiedImages);

  mDetector.setDictionary(dictionary_type);

  if (parent_name == "" || child_name == "")
  {
    RCLCPP_ERROR(node->get_logger(), "parent_name and/or child_name was not set!");
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "Image is rectified: %s", useRectifiedImages ? "true" : "false");

  image_transport::Subscriber image_sub = it.subscribe("/image_rect", 1, &image_callback);
  cam_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", 1, &cam_info_callback);
  cam_info_received = false;
  image_pub = it.advertise("result", 10);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}