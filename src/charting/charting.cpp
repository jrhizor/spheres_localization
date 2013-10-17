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

struct RegImg
{
  std::string rgb_file;
  std::string depth_file;
  cv::Mat transformation;
};

std::vector<RegImg> load_sequence(const std::string &input_file)
{
  std::vector<RegImg> reg_imgs;
  std::ifstream fin(input_file.c_str());
  unsigned int num_img;

  fin >> num_img;

  for(unsigned int i=0; i<num_img; ++i)
  {
    RegImg img;
    cv::Mat transformation(3, 3, CV_32S);

    std::getline(fin, img.rgb_file);
    std::getline(fin, img.depth_file);

    fin >> transformation.at<float>(1, 1)
        >> transformation.at<float>(1, 2)
        >> transformation.at<float>(1, 3)
        >> transformation.at<float>(2, 1)
        >> transformation.at<float>(2, 2)
        >> transformation.at<float>(2, 3)
        >> transformation.at<float>(3, 1)
        >> transformation.at<float>(3, 2)
        >> transformation.at<float>(3, 3);

    img.transformation = transformation;

    reg_imgs.push_back(img);
  }

  fin.close();

  return reg_imgs;
}

std::vector<std::vector<double> > generate_3d_desc(const RegImg &img, const std::string &type, int type_size)
{
  cv::Mat rgb = cv::imread(img.rgb_file, CV_LOAD_IMAGE_COLOR);
  cv::Mat depth = cv::imread(img.depth_file, CV_LOAD_IMAGE_GRAYSCALE);

  ROS_ASSERT(rgb.rows != 0 && rgb.cols != 0 && depth.rows != 0 && depth.cols != 0);

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat desc;

  if(type.compare(std::string("ORB"))==0)
  {
    cv::ORB orbDect(700, 1.2f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31); 
    orbDect.detect(rgb, keypoints);
    orbDect.compute(rgb, keypoints, desc);
  }
  else if(type.compare(std::string("SIFT"))==0)
  {
    cv::SiftFeatureDetector siftDect; 
    siftDect.detect(rgb, keypoints);
    siftDect.compute(rgb, keypoints, desc);
  }
  else if(type.compare(std::string("SURF"))==0)
  {
    cv::SURF surfDect(100, 5,1,true, false);
    surfDect(rgb, cv::noArray(), keypoints, desc);
  }
  else if(type.compare(std::string("FAST+FREAK"))==0)
  {
    cv::FREAK freakDect;  
    cv::FAST(rgb, keypoints, 20/*threshold*/);
    freakDect.compute(rgb, keypoints, desc);
  }
  else if(type.compare(std::string("MSER+ORB"))==0)
  {
    cv::Ptr<cv::FeatureDetector> detector = new cv::MserFeatureDetector();
    cv::ORB orbDect(700, 1.2f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31); 
    detector->detect(rgb, keypoints);
    orbDect.compute(rgb, keypoints, desc);
  } 
  else if(type.compare(std::string("FAST+ORB"))==0)
  {
    cv::ORB orbDect(700, 1.2f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31); 
    cv::FAST(rgb, keypoints, 20/*threshold*/);
    orbDect.compute(rgb, keypoints, desc);
  }
  else
  {
    ROS_ERROR("The descriptor-detector pair type is invalid.");
    exit(-1);
  }

  std::vector<std::vector<double> > map_elements;

  for(unsigned int i=0; i<keypoints.size(); ++i)
  {
    // TODO: check if u and v are the right order
    int u = keypoints[i].pt.x;
    int v = keypoints[i].pt.y;

    pcl::PointXYZ pos = get_3d_point(depth, u, v);
    cv::Mat mat_pos(3, 1, CV_32S);

    // TODO: check if this has to be in homogenous coordinates
    mat_pos.at<float>(1,1) = pos.x;
    mat_pos.at<float>(2,1) = pos.y;
    mat_pos.at<float>(3,1) = pos.z;

    cv::Mat trans_pos = img.transformation * mat_pos;

    std::vector<double> map_element;

    map_element.push_back(trans_pos.at<float>(1,1));
    map_element.push_back(trans_pos.at<float>(2,1));
    map_element.push_back(trans_pos.at<float>(3,1));

    // TODO: check if this is grabbing the correct part of desc
    for(unsigned int j=0; j<type_size; ++j)
    {
      map_element.push_back(desc.at<float>(i, j));
    }

    map_elements.push_back(map_element);
  }

  return map_elements;
}

void generate_map(const std::vector<RegImg> &reg_imgs, const std::string &output_file, 
                  const std::string &type, int type_size)
{
  std::ofstream fout(output_file.c_str());

  fout << reg_imgs.size() << std::endl
       << type_size << std::endl;

  fout.precision(15);

  for(unsigned int i=0; i<reg_imgs.size(); ++i)
  {
    std::vector<std::vector<double> > output_lines = generate_3d_desc(reg_imgs[i], type, type_size);
    
    for(unsigned int j=0; j<output_lines.size(); ++j)
    {
      ROS_ASSERT(output_lines[j].size()-3==type_size);

      for(unsigned int k=0; k<3; ++k)
      {
        fout << output_lines[j][k] << " ";
      }

      fout << std::endl;

      for(unsigned int k=3; k<output_lines[j].size(); ++k)
      {
        fout << output_lines[j][k] << " ";
      }

      fout << std::endl;
    }
  }

  fout.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "charting");

  ROS_ASSERT(argc==5);

  std::string input_file = argv[1]; // .slmapreg file
  std::string output_file = argv[2]; // .slmap file
  std::string type = argv[3]; // descriptor type name
  int type_size = atoi(argv[4]); // descriptor type name

  std::vector<RegImg> reg_imgs = load_sequence(input_file);

  generate_map(reg_imgs, output_file, type, type_size);

  return 0;
}


