#include <stdint.h>

#include <string>
#include <sstream>

#include <cassert>
#include <functional>

#include <stdio.h>
#include <time.h>
#include <ctime>
#include <vector>
#include <cmath>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

#include <fstream>

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/lexical_cast.hpp>

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;


class T0
{
  public:
    T0();
    void update_rgb(const sensor_msgs::ImageConstPtr& msg);
    void update_d(const sensor_msgs::ImageConstPtr& msg);
    void capture_frame();   
    
    bool running;

  private: 
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub_rgb, sub_d;
    cv_bridge::CvImagePtr rgb, d;
    cv_bridge::CvImage rgbI, dI;

    vector<string> timestamps;
    vector<Mat> rgb_data, depth_data;
};

T0::T0() : it(n), running(true)
{
  sub_rgb = it.subscribe("/camera/rgb/image_color", 1, &T0::update_rgb, this);
  sub_d = it.subscribe("/camera/depth_registered/image_rect", 1, &T0::update_d, this);

  // ros::Time scan_time = ros::Time::now();
}

void T0::capture_frame()
{
  ros::spinOnce();  

  // capture rgb and depth images

  // transform them to Mat
  // Mat rgb_frame = rgbI;
  // Mat depth_frame = dI;

  // // save to vectors
  // rgb_data.push_back(rgb_frame);
  // depth_data.push_back(depth_frame);  

  string timestamp = boost::lexical_cast<std::string>(ros::Time::now().toNSec());
  timestamp.insert(timestamp.end()-9, 1, '.');
  timestamps.push_back(timestamp);

  ROS_INFO("Frame captured."); 
}

void T0::update_rgb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    rgb = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  rgbI = *rgb;
}

void T0::update_d(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    d = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  dI = *d;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "T0");
  T0 program;

  ROS_INFO("T0 Starting.");   

  if(argc!=2)
  {
    ROS_ERROR("No output name was specified.");
    return 1;
  }

  ROS_INFO("Press enter to capture a frame. Type \"done\" to save your dataset. Type \"q\" to quit without saving.");
  
  while(ros::ok() && program.running)
  {
    string key;
    getline( cin, key );

    // check for escape key
    if(key.compare("done")==0)
    {
      program.running = false;
      ROS_INFO("Ending data collection.");                                            
    }
    else if(key.compare("q")==0 || key.compare("quit")==0)
    {
      ROS_INFO("Quitting without saving.");
    }
    else
    {
      program.capture_frame();             
    }
  }

  // save dataset
  

  return 0; 
}
