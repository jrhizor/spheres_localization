#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "temp_broadcaster");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("temp_broadcaster", 1000);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "1.64429 1.50519 -0.39932 0.594102 -0.164865 0.787314 0.0784316 0.985981 0.147282 -0.800558 -0.0257501 0.598703";
    msg.data = ss.str();




    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}