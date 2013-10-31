// CHARTING NEEDS COMPREHENSIVE TESTING

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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <spheres_localization/utilities/utilities.h>
#include <spheres_localization/utilities/registered_maps.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "charting");

  ROS_ASSERT(argc==6);

  std::string input_folder = argv[1]; 
  int num_images = atoi(argv[2]);
  std::string desc_type = argv[3]; // descriptor type name
  int desc_type_size = atoi(argv[4]); // descriptor type size
  std::string output_path = argv[5];

  // force trailing slash
  if(input_folder[input_folder.size()-1]!='/')
  {
    input_folder = input_folder + std::string("/");
  }

  std::cout << "Loading sequence." << std::endl;
  std::vector<RegImg> reg_imgs = load_sequence(input_folder, num_images);


  std::cout << "Generating map." << std::endl;
  generate_map(reg_imgs, output_path, desc_type, desc_type_size);

  std::cout << "Finished." << std::endl;

  return 0;
}


