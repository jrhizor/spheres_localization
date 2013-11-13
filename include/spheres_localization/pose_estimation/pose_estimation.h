#ifndef SL_POSE_ESTIMATION_H
#define SL_POSE_ESTIMATION_H

#include <stdint.h>

#include <string>
#include <map>
#include <sstream>

#include <cassert>
#include <functional>

#include <stdio.h>
#include <time.h>
#include <cstdlib>
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

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>

#include <boost/shared_array.hpp>
#include <boost/math/quaternion.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <spheres_localization/utilities/rottoquat.h>
#include <spheres_localization/utilities/registered_maps.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

namespace enc = sensor_msgs::image_encodings;

typedef std::vector<InterestPoint3D> PtLookupTable;

bool first = true;

void findMatchesAndPose(cv::Mat &desc, cv::Mat &desc2, const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::KeyPoint> &keypoints2, 
            int &numInliers, 
            int &numGoodMatches, int &timeMatch, int &timePE, const cv::Mat &queryImg,
            cv::Mat &tvec, ::boost::math::quaternion<double> &q, PtLookupTable &map_position_lookup, cv::Mat &rot_mat_result,cv::Mat &rvec);

int pnp(const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::KeyPoint> &keypoints2, 
            const std::vector<cv::DMatch> &good_matches, 
            cv::Mat &tvec, ::boost::math::quaternion<double> &q, PtLookupTable &map_position_lookup, cv::Mat &rot_mat_result,cv::Mat &rvec);

void getFeatures(const std::string &method, const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &desc, int &timeDetect, 
            int &timeDescribe)
{
  if(method.compare(std::string("ORB"))==0)
  {
    cv::ORB orbDect(700, 1.2f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31); 
    int startMark = clock();
    orbDect.detect(img, keypoints);
    int detectMark = clock();
    orbDect.compute(img, keypoints, desc);
    int describeMark = clock();

    // record times
    timeDetect = detectMark - startMark;
    timeDescribe = describeMark - detectMark;
  }
  else if(method.compare(std::string("SIFT"))==0)
  {
    keypoints.clear();
    desc.release();

    cv::SiftFeatureDetector siftDect; 
    int startMark = clock();
    siftDect.detect(img, keypoints);
    int detectMark = clock();
    siftDect.compute(img, keypoints, desc);
    int describeMark = clock();

    // record times
    timeDetect = detectMark - startMark;
    timeDescribe = describeMark - detectMark;
  }
  else if(method.compare(std::string("SURF"))==0)
  {
    keypoints.clear();
    desc.release();

    cv::SURF surfDect(100, 5,1,true, false);
    int startMark = clock();
    surfDect(img, cv::noArray(), keypoints, desc);
    int describeMark = clock();

    // record times
    timeDetect =  (describeMark - startMark)/2.0;
    timeDescribe = (describeMark - startMark)/2.0;
  }
  else if(method.compare(std::string("FAST+FREAK"))==0)
  {
    cv::FREAK freakDect;  
    int startMark = clock();
    cv::FAST(img, keypoints, 20//threshold
    );
    int detectMark = clock();
    freakDect.compute(img, keypoints, desc);
    int describeMark = clock();

    // record times
    timeDetect = detectMark - startMark;
    timeDescribe = describeMark - detectMark;
  }
  else if(method.compare(std::string("MSER+ORB"))==0)
  {
    cv::Ptr<cv::FeatureDetector> detector = new cv::MserFeatureDetector();
    cv::ORB orbDect(700, 1.2f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31); 

    int startMark = clock();
    detector->detect( img, keypoints );
    int detectMark = clock();
    orbDect.compute(img, keypoints, desc);
    int describeMark = clock();

    // record times
    timeDetect = detectMark - startMark;
    timeDescribe = describeMark - detectMark;
  } 
  else if(method.compare(std::string("FAST+ORB"))==0)
  {
    cv::ORB orbDect(700, 1.2f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31); 
    int startMark = clock();
    cv::FAST(img, keypoints, 20 //threshold
      );
    int detectMark = clock();
    orbDect.compute(img, keypoints, desc);
    int describeMark = clock();

    // record times
    timeDetect = detectMark - startMark;
    timeDescribe = describeMark - detectMark;
  }
  else
  {
    std::cout << "NO VALID METHOD SELECTED" << std::endl;
    exit(-1);
  }
}

void findMatchesAndPose(cv::Mat &desc, cv::Mat &desc2, const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::KeyPoint> &keypoints2, 
             int &numInliers, 
            int &numGoodMatches, int &timeMatch, int &timePE, const cv::Mat &queryImg,
            cv::Mat &tvec, ::boost::math::quaternion<double> &q, PtLookupTable &map_position_lookup, cv::Mat &rot_mat_result,cv::Mat &rvec)
{
  cv::BruteForceMatcher<cv::L2<float> > matcher;
  std::vector<std::vector<cv::DMatch> > matches;
  int startMark, endMark;

  if(desc.type()!=CV_32F) {
      desc.convertTo(desc, CV_32F);
  }

  if(desc2.type()!=CV_32F) {
      desc2.convertTo(desc2, CV_32F);
  }
    
    startMark = clock();

    matcher.knnMatch(desc, desc2, matches, 2);

  std::cout << "desc.height " << desc.size().height << std::endl;
  std::cout << "desc.width " << desc.size().width << std::endl; 
  std::cout << "desc2.height " << desc2.size().height << std::endl;
  std::cout << "desc2.width " << desc2.size().width << std::endl; 

  std::cout << "matches.size " << matches.size() << std::endl;

  double ratio = 0.75;
  std::vector<cv::DMatch > good_matches;
  for(int i = 0; i < matches.size(); i++)
  {
    if(matches[i].size() == 2 && 
          (matches[i][0].distance / matches[i][1].distance)<ratio // &&
          // keypoints2[matches[i][0].queryIdx].pt.y <480 &&
          // keypoints2[matches[i][0].queryIdx].pt.x <640 &&
          // keypoints2[matches[i][0].queryIdx].pt.y >=0 &&
          // keypoints2[matches[i][0].queryIdx].pt.x >=0)
          )
    {
      good_matches.push_back(matches[i][0]);
    }
  }

  std::cout << "good_matches.size " << good_matches.size() << std::endl;

  if(matches.size()==0)
  {
    std::cout << "0 matches" <<std::endl;
    exit(-1);
  }

  endMark = clock();
  timeMatch = endMark - startMark;


  startMark = clock();
  // std::cout << "BEFORE EPNP" << std::endl;
  numInliers = pnp(keypoints, keypoints2, good_matches, tvec, q, map_position_lookup, rot_mat_result, rvec);

  endMark = clock();

  timePE = endMark - startMark;

  numGoodMatches = good_matches.size();

}



int pnp(const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::KeyPoint> &keypoints2, 
            const std::vector<cv::DMatch> &good_matches,
            cv::Mat &tvec, ::boost::math::quaternion<double> &q, PtLookupTable &map_position_lookup, cv::Mat &rot_mat_result, cv::Mat &rvec)
{
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;
  
  cv::Vec3d euler;

  for(int i = 0; i < good_matches.size(); i++)
  {
    // std::pair<float,float> location = std::make_pair(keypoints[(good_matches[i].queryIdx)].pt.x,keypoints[(good_matches[i].queryIdx)].pt.y);
    // pcl::PointXYZ pt = map_position_lookup[location];

    InterestPoint3D ipt = map_position_lookup[good_matches[i].queryIdx];
    pcl::PointXYZ pt(ipt.x, ipt.y, ipt.z);

    double Xw = pt.x, Yw = pt.y, Zw = pt.z, u, v;

    u = keypoints2[(good_matches[i].trainIdx)].pt.x;
    v = keypoints2[(good_matches[i].trainIdx)].pt.y;

    if(!(Zw != Zw))
    {
      objectPoints.push_back(cv::Point3f(Xw, Yw, Zw));
      imagePoints.push_back(cv::Point2f(u,v));
    }
  }


std::cout << good_matches.size() << std::endl;

  cv::Mat rotmat, jacobian;

  // default for
  cv::Matx33f cameraMatrix(525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0);

  std::vector<float> distortions;
  std::vector<int> inliers;

  //solvePnP(objectPoints, imagePoints, cameraMatrix, distortions, rvec, tvec, false, CV_EPNP);
  
  int initMinInliers = 200;
  int count = 0;

  while(inliers.size()==0 && count < 20)
  {
    inliers.clear();
    solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distortions, rvec, tvec, false, 
          2000, //iterations 
          10, // reproj error 
          initMinInliers, // min inliers 
          inliers, CV_EPNP);
 
    if(first) first = false;

    if(initMinInliers<imagePoints.size()*.75) initMinInliers++;
    count++;
  }
  

  Rodrigues(rvec, rotmat, jacobian);

  // convert rotmat into quaternion
  R3_matrix<double> rot_mat_r3;
  rot_mat_r3.a11 = rotmat.at<double>(0,0);
  rot_mat_r3.a12 = rotmat.at<double>(0,1);
  rot_mat_r3.a13 = rotmat.at<double>(0,2);
  rot_mat_r3.a21 = rotmat.at<double>(1,0);
  rot_mat_r3.a22 = rotmat.at<double>(1,1);
  rot_mat_r3.a23 = rotmat.at<double>(1,2);
  rot_mat_r3.a31 = rotmat.at<double>(2,0);
  rot_mat_r3.a32 = rotmat.at<double>(2,1);
  rot_mat_r3.a33 = rotmat.at<double>(2,2);

  cv::Mat rot_mat_temp(3, 3, CV_32F);
  for(int i=0; i<3; i++)
  {   
    for(int j=0; j<3; j++)
    {
      rot_mat_temp.at<float>(i,j) = float(rotmat.at<double>(i,j));
    }
  }
  rot_mat_result = rot_mat_temp.inv();

  tvec.at<double>(0) *= -1;
  tvec.at<double>(1) *= -1;
  tvec.at<double>(2) *= -1;

   std::cout << "1-------------------------------1" << std::endl;
      std::stringstream ss;
      ss << tvec.at<double>(0) << " "
         << tvec.at<double>(1) << " "
         << tvec.at<double>(2) << " "
         << rot_mat_result.at<float>(0,0) << " "
         << rot_mat_result.at<float>(0,1) << " "
         << rot_mat_result.at<float>(0,2) << " "
         << rot_mat_result.at<float>(1,0) << " "
         << rot_mat_result.at<float>(1,1) << " "
         << rot_mat_result.at<float>(1,2) << " "
         << rot_mat_result.at<float>(2,0) << " "
         << rot_mat_result.at<float>(2,1) << " "
         << rot_mat_result.at<float>(2,2);


      std::cout << ss.str() << std::endl << std::endl;
   std::cout << "2-------------------------------2" << std::endl;


  q = R3_rotation_to_quaternion(rot_mat_r3);

  euler = euler_angle(rotmat);


  // todo: check ordering
  // std::cout << "roll" << "\t\t" << "pitch" << "\t\t" << "yaw" << std::endl;
  // std::cout << euler[0] <<"\t" << euler[1] << "\t" << euler[2] << std::endl; 
  // std::cout << tvec << std::endl;
  //std::cout << inliers.size() << std::endl;

  return inliers.size();
}


class PoseEstimator
{
  public:
    PoseEstimator(const std::string &camera_topic);
    void updateRGB(const sensor_msgs::ImageConstPtr& msg);
    void run(std::vector<cv::KeyPoint> &map_keypoints, cv::Mat &map_desc, PtLookupTable &map_position_lookup, const std::string &method);

  private: 
    // ros
    ros::NodeHandle n;

    // subscribing to camera
    image_transport::ImageTransport it;
    image_transport::Subscriber rgb_sub;
    cv_bridge::CvImagePtr rgb;
    cv_bridge::CvImage rgbI;

    // publishing to visualizer
    ros::Publisher pose_pub;

    // handle updates
    bool new_image;
};

void PoseEstimator::updateRGB(const sensor_msgs::ImageConstPtr& msg)
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

  new_image = true;
}

PoseEstimator::PoseEstimator(const std::string &camera_topic) : it(n), new_image(false)
{
  rgb_sub = it.subscribe(camera_topic, 1, &PoseEstimator::updateRGB, this);
  pose_pub = n.advertise<std_msgs::String>("pose_estimation", 1000);
}

void PoseEstimator::run(std::vector<cv::KeyPoint> &map_keypoints, cv::Mat &map_desc, PtLookupTable &map_position_lookup, const std::string &method)
{
  // initialize stuff
  int timeDetect, timeDescribe, timeMatch, timePE;
  int totalDetect=0, totalDescribe=0, totalMatch=0, totalInliers=0, totalGoodMatches=0, totalPE = 0,
      numGoodMatches=0, numInliers=0;
  int numQueries = 12;

  cv::Mat tvec, rvec, rot_mat;
  ::boost::math::quaternion<double> q;

  int counter = 0;


  while(ros::ok())
  {
    ros::spinOnce();
    
    // std::string queryFile = "/home/jrhizor/Desktop/KinFuSnapshots/0.png";
    // cv::Mat img = cv::imread(queryFile, 0); 
    //std::stringstream newss;
    //newss << counter;
    //cv::Mat img = cv::imread("rot/" + newss.str() + ".png", CV_LOAD_IMAGE_GRAYSCALE); //rgbI.image;
    cv::Mat img = rgbI.image;
    //counter++;
    //counter %= 12;


    if(img.size().height !=0)
    {
      new_image = false;

      std::vector<cv::KeyPoint> keypoints;
      cv::Mat desc;

      getFeatures(method, img, keypoints, desc, timeDetect, timeDescribe);

      findMatchesAndPose(map_desc, desc, map_keypoints, keypoints, numInliers, numGoodMatches, 
                timeMatch, timePE, img, tvec, q, map_position_lookup, rot_mat, rvec);

      // output pose 
      // std::cout  << " " <<
      //       -1*tvec.at<double>(0,0) << " " << -1*tvec.at<double>(0,1) << " " << -1*tvec.at<double>(0,2) << " " <<
      //       q.R_component_1() <<" "<< q.R_component_2()  << " " << q.R_component_3() << " " << q.R_component_4() << 

    cv::Mat tvec_t(3, 1, CV_32FC1);

    tvec_t.at<float>(0,0) = tvec.at<double>(0);
    tvec_t.at<float>(1,0) = tvec.at<double>(1);
    tvec_t.at<float>(2,0) = tvec.at<double>(2);

      cv::Mat tvec_result =  rot_mat * tvec_t;


    tvec.at<double>(0) = tvec_result.at<float>(0,0);
    tvec.at<double>(1) = tvec_result.at<float>(1,0);
    tvec.at<double>(2) = tvec_result.at<float>(2,0);

      std::cout << "tvec " << tvec << std::endl;
      std::cout << "rot_mat " << rot_mat << std::endl;

      std::stringstream ss;
      ss << tvec.at<double>(0) << " "
         << tvec.at<double>(1) << " "
         << tvec.at<double>(2) << " "
         << rot_mat.at<float>(0,0) << " "
         << rot_mat.at<float>(0,1) << " "
         << rot_mat.at<float>(0,2) << " "
         << rot_mat.at<float>(1,0) << " "
         << rot_mat.at<float>(1,1) << " "
         << rot_mat.at<float>(1,2) << " "
         << rot_mat.at<float>(2,0) << " "
         << rot_mat.at<float>(2,1) << " "
         << rot_mat.at<float>(2,2);


      std::cout << ss.str() << std::endl << std::endl;

      std_msgs::String msg;
      msg.data = ss.str();
      pose_pub.publish(msg);
    }
  }
}

#endif