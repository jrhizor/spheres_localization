#ifndef SL_REGISTERED_MAPS_H
#define SL_REGISTERED_MAPS_H


#include <tr1/tuple>

#include <stdint.h>

#include <string>
#include <sstream>
#include <map>

#include <cassert>
#include <functional>

#include <stdio.h>
#include <time.h>
#include <ctime>
#include <vector>
#include <cmath>
#include <iostream>
#include <signal.h>
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
#include <spheres_localization/utilities/utilities.h>

typedef std::map<std::tr1::tuple<int,int,int>, std::vector<std::vector<std::vector<double> > > > map_pt_desc;

struct InterestPoint3D
{
  float x,y,z;
  std::vector<float> descriptor;
};

struct RegImg
{
  std::string rgb_file;
  std::string depth_file;
  cv::Mat translation, rotation;
};

std::vector<RegImg> load_sequence(const std::string &input_folder, int num_images)
{
  std::vector<RegImg> reg_imgs;

  for(int i=0; i<num_images; ++i)
  {
    std::cout << "\tProcessing frame " << i << std::endl;

    RegImg frame;

    // record file names
    frame.rgb_file = input_folder + boost::lexical_cast<std::string>(i) + std::string(".png");
    frame.depth_file = input_folder + boost::lexical_cast<std::string>(i) + std::string("_d.png");
    std::string transform_file = input_folder + boost::lexical_cast<std::string>(i) + std::string(".txt");

    std::cout << "\t\tRGB File " << frame.rgb_file << std::endl;
    std::cout << "\t\tDepth File " << frame.depth_file << std::endl;
    std::cout << "\t\tTransform File " << transform_file << std::endl;

    // load transformation
    std::ifstream fin(transform_file.c_str());

    cv::Mat rotation(3, 3, CV_32FC1);
    cv::Mat translation(3, 1, CV_32FC1);

    std::string temp;

    fin >> temp;

    fin >> translation.at<float>(0, 0) 
        >> translation.at<float>(1, 0) 
        >> translation.at<float>(2, 0);

    fin >> temp;

    fin >> rotation.at<float>(0, 0)
        >> rotation.at<float>(0, 1)
        >> rotation.at<float>(0, 2)
        >> rotation.at<float>(1, 0)
        >> rotation.at<float>(1, 1)
        >> rotation.at<float>(1, 2)
        >> rotation.at<float>(2, 0)
        >> rotation.at<float>(2, 1)
        >> rotation.at<float>(2, 2);

    fin.close();

    frame.translation = translation;
    frame.rotation = rotation;

    reg_imgs.push_back(frame);
  }

  return reg_imgs;
}


std::vector<std::vector<double> > generate_3d_desc(const RegImg &img, const std::string &type, int type_size)
{
  cv::Mat rgb = cv::imread(img.rgb_file, CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat depth = cv::imread(img.depth_file, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);

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
    int u = keypoints[i].pt.x;
    int v = keypoints[i].pt.y;

    pcl::PointXYZ pos = get_3d_point(depth, u, v);
    cv::Mat mat_pos(3, 1, CV_32FC1);

    mat_pos.at<float>(0,0) = pos.x;
    mat_pos.at<float>(1,0) = pos.y;
    mat_pos.at<float>(2,0) = pos.z;

    //cv::Mat trans_pos = (img.rotation.inv()*img.translation)+(img.rotation *mat_pos);
    //cv::Mat trans_pos =(img.translation)+(img.rotation.inv() *mat_pos);
    
    // Pure camera positioning
    //cv::Mat trans_pos =(img.translation);

    // Positioning + map data
    cv::Mat trans_pos =(img.translation)+(img.rotation *mat_pos);


    //cv::Mat trans_pos = (img.rotation.inv()*img.translation);

    //cv::Mat trans_pos = mat_pos + img.rotation.inv() * img.translation;
    // cv::Mat trans_pos = (img.rotation*img.translation) + mat_pos;

    std::vector<double> map_element;

    map_element.push_back(trans_pos.at<float>(0,0));
    map_element.push_back(trans_pos.at<float>(1,0));
    map_element.push_back(trans_pos.at<float>(2,0));

    for(unsigned int j=0; j<type_size; ++j)
    {
      map_element.push_back(desc.at<float>(i, j));
    }

    map_elements.push_back(map_element);
  }

  std::cout << "\t\tDescriptors: " << map_elements.size() << std::endl;

  return map_elements;
}

// float dist(const std::vector<double> &pt1, const std::vector<double> &pt2)
// {
//   return sqrt((pt1[0]-pt2[0])*(pt1[0]-pt2[0])+(pt1[1]-pt2[1])*(pt1[1]-pt2[1])+(pt1[2]-pt2[2])*(pt1[2]-pt2[2]));
// }

float desc_dist(const std::vector<double> &pt1, const std::vector<double> &pt2)
{
  float sum = 0;

  for(unsigned int i=3; i<pt1.size(); ++i)
  {
    sum += (pt1[i]-pt2[i])*(pt1[i]-pt2[i]);
  }

  return sqrt(sum);
}

std::vector<double> average_pt(const std::vector<std::vector<double> > &group)
{
  std::vector<double> avg(group[0].size(),0);

  for(unsigned int i=0; i<group.size(); ++i)
  {
    for(unsigned int j=0; j<group[i].size(); ++j)
    {
      avg[j] += group[i][j];
    }
  }

  for(unsigned int i=0; i<avg.size(); ++i)
  {
    avg[i] /= group.size();
  }

  return avg;
}

void generate_map(const std::vector<RegImg> &reg_imgs, const std::string &output_file, 
                  const std::string &type, int type_size, float max_desc_error)
{
  std::vector<std::vector<double> > output;

  // compute all descriptors
  for(unsigned int i=0; i<reg_imgs.size(); ++i)
  {
    std::cout << "\tProcessing frame " << i << std::endl;
    std::vector<std::vector<double> > output_lines = generate_3d_desc(reg_imgs[i], type, type_size);
    output.insert(output.end(), output_lines.begin(), output_lines.end());
  }

  std::cout << std::endl << "Number of points before handling similar points: " << output.size() << std::endl;

  // handle similar points
  std::vector<std::vector<double> >::iterator it = output.begin(); 
  map_pt_desc hash;
  while(it != output.end())
  {
    std::tr1::tuple<int,int,int> approx_loc(round(100*((*it)[0]+0.000001)),round(100*((*it)[1]+0.000001)),round(100*((*it)[2]+0.000001)));

    // if this location isn't found
    if(hash.find(approx_loc) == hash.end()) 
    {
      std::vector<std::vector<std::vector<double> > > new_entry;
      std::vector<std::vector<double> > new_category;
      new_category.push_back(*it);
      new_entry.push_back(new_category);

      hash[approx_loc] = new_entry;
    } 

    // this location already exists
    else 
    {
      bool added = false;

      for(unsigned int i=0; i<hash[approx_loc].size(); ++i)
      {
        if(desc_dist(hash[approx_loc][i][0], *it)<max_desc_error)
        {
          added = true;

          hash[approx_loc][i].push_back(*it);
        }
      }

      if(!added)
      {
        std::vector<std::vector<double> > new_category;
        new_category.push_back(*it);

        hash[approx_loc].push_back(new_category);
      }
    }

    it++;
  }

  std::cout << std::endl << "Number of locations after handling similar points: " << hash.size() << std::endl;

  // output to file
  std::ofstream fout(output_file.c_str());

  fout << reg_imgs.size() << std::endl
       << type_size << std::endl;

  fout.precision(15);

  int counter = 0;

  // for each location
  for(map_pt_desc::iterator it = hash.begin(); it != hash.end(); it++)
  { 
    // for each group
    for(unsigned int i=0; i < it->second.size(); ++i)
    {
      counter++;

      std::vector<double> avg = average_pt(it->second[i]);

      for(unsigned int k=0; k<3; ++k)
      {
        fout << avg[k] << " ";
      }

      fout << std::endl;

      for(unsigned int k=3; k<avg.size(); ++k)
      {
        fout << avg[k] << " ";
      }

      fout << std::endl;
    }
  }

  std::cout << std::endl << "Number of unique points after handling similar points: " << counter << std::endl;

  fout.close();
}

std::vector<InterestPoint3D> load_map(const std::string &input)
{
  std::vector<InterestPoint3D> point_map;

  std::ifstream fin(input.c_str());

  int num_img, type_size;

  fin >> num_img >> type_size;

  InterestPoint3D pt;
  fin >> pt.x >> pt.y >> pt.z;

  while(fin.good())
  {
    pt.descriptor.clear();
    
    for(unsigned int j=0; j<type_size; ++j)
    {
      float temp;
      fin >> temp;
      pt.descriptor.push_back(temp);
    }

    point_map.push_back(pt);

    fin >> pt.x >> pt.y >> pt.z;
  }

  fin.close();

  return point_map;
}

#endif