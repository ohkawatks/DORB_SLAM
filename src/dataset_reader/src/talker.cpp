#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub_img; 
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  pub_img_temp_(it.advertise(USB_CAM_TEMP, 1000)),
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    ROS_INFO("%s", msg.data.c_str());
    //    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
