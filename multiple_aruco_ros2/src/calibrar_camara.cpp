#include <ros/ros.h>
#include <image_transport/image_transport.h>//Librería de image transport
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::VideoCapture cap;//Se declara una variable para la captura de video
cv::Mat frame;//Se declara una variable tipo mat
sensor_msgs::ImagePtr msg;//Se declara el tipo de mensaje que se transmitirá en el tópico

image_transport::Publisher pub;//Se declara el publisher

void timerCallback(const ros::TimerEvent&)//Se inicia la función que se manda a llamar desde el timer
{  
      cap >> frame; //Se guarda la imagen del vídeo en la variable tipo mat
	if(!frame.empty()) {//Se verifica que la matriz no esté vacía
  	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();//Se convierte la información de la variable mat a tipo de sensor_msgs, se puede escoger el tipo de compresión y en este caso se ha declarado como rgb de 8 bits
  	pub.publish(msg);//Se publica la imagen
  	//cv::imshow("Window", frame);
  	cv::waitKey(1);//Da tiempo para dibujar la imagen
	}

	ros::spinOnce();  //Se actauliza la información del nodo
}


int main(int argc, char** argv)//Función principal
{
  ros::init(argc, argv, "cv_template_video");//Inicialización del nodo y se le da nombre
  ros::NodeHandle nh; //Declaracipon del nodehandle
  image_transport::ImageTransport it(nh);//Se utiliza image transport en el nodo para poder publicar la imagen
  pub = it.advertise("usb_cam/image_raw", 1);//Se nombra el tópico en el que se va a publicar
  ROS_INFO("Start cv_template_video");
 
  //cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cap.open(0, cv::CAP_ANY);//Se declara la cámara que se ocupará

  if (!cap.isOpened())  
  {
  	ROS_INFO("ERROR! Unable to open camera");
  	return 1;
  }

  ROS_INFO("Start grabbing...");
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/60.0), timerCallback); //Se manda a llamar e¿la función del timer
  ros::spin();//Se actualiza la información del nodo
}

