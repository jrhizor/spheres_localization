#ifndef SL_POSE_ESTIMATION_H
#define SL_POSE_ESTIMATION_H

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

void findMatchesAndPose(cv::Mat &desc, cv::Mat &desc2, const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::KeyPoint> &keypoints2, 
            const cv::Mat &depth, int &numInliers, 
            int &numGoodMatches, int &timeMatch, int &timePE, const cv::Mat &mapImg, const cv::Mat &queryImg,
            cv::Mat &tvec, ::boost::math::quaternion<double> &q);

int pnp(const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::KeyPoint> &keypoints2, 
            const std::vector<cv::DMatch> &good_matches, const cv::Mat &depth, 
            cv::Mat &tvec, ::boost::math::quaternion<double> &q);

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
            const cv::Mat &depth, int &numInliers, 
            int &numGoodMatches, int &timeMatch, int &timePE, const cv::Mat &mapImg, const cv::Mat &queryImg,
            cv::Mat &tvec, ::boost::math::quaternion<double> &q)
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

  double ratio = 0.75;
  std::vector<cv::DMatch > good_matches;
  for(int i = 0; i < matches.size(); i++)
  {
      if(matches[i].size() == 2 && 
          (matches[i][0].distance / matches[i][1].distance)<ratio // && good_matches.size() <20
          &&
          keypoints2[matches[i][0].queryIdx].pt.y <480 &&
          keypoints2[matches[i][0].queryIdx].pt.x <640 &&
          keypoints2[matches[i][0].queryIdx].pt.y >=0 &&
          keypoints2[matches[i][0].queryIdx].pt.x >=0)
    {
      float x = (float) depth.at<int16_t>(int(keypoints2[matches[i][0].queryIdx].pt.y),int(keypoints2[matches[i][0].queryIdx].pt.x));
      if(!(x != x) || x == 0)
        {
          good_matches.push_back(matches[i][0]);
        }
    }
  }

  if(matches.size()==0)
  {
    std::cout << "0 matches" <<std::endl;
    exit(-1);
  }

  endMark = clock();
  timeMatch = endMark - startMark;


cv::Mat result;
      drawMatches(mapImg, keypoints, queryImg, keypoints2, good_matches, result, cv::Scalar::all(-1), cv::Scalar::all(-1),
                std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS  ); 
   std::stringstream ss;//create a stringstream
   ss << "/home/jared/Desktop/spheres_localization/kinect_report/results/" << clock() << ".jpg";//add number to the stream

   std::cout << ss.str() << std::endl;
  //namedWindow(ss.str(), CV_WINDOW_AUTOSIZE );// Create a window for display.
  imwrite(ss.str(), result);
//     imshow( ss.str(), result );   
// char  aksk;
// cin >> aksk;


  startMark = clock();
  std::cout << "BEFORE EPNP" << std::endl;
  numInliers = pnp(keypoints, keypoints2, good_matches, depth, tvec, q);
  endMark = clock();

  timePE = endMark - startMark;

  numGoodMatches = good_matches.size();

}




int pnp(const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::KeyPoint> &keypoints2, 
            const std::vector<cv::DMatch> &good_matches, const cv::Mat &depth, 
            cv::Mat &tvec, ::boost::math::quaternion<double> &q)
{
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;
  
  cv::Vec3d euler;

    for(int i = 0; i < good_matches.size(); i++)
  {
    pcl::PointXYZ pt = get_3d_point(depth,keypoints[(good_matches[i].queryIdx)].pt.x, 
                    keypoints[(good_matches[i].queryIdx)].pt.y);

      double Xw = pt.x, Yw = pt.y, Zw = pt.z, u, v;

    u = keypoints2[(good_matches[i].trainIdx)].pt.x;
    v = keypoints2[(good_matches[i].trainIdx)].pt.y;

      if(!(Zw != Zw))
      {
      objectPoints.push_back(cv::Point3f(Xw, Yw, Zw));
      imagePoints.push_back(cv::Point2f(u,v));
      }
  }

  cv::Mat rvec, rotmat, jacobian;

  // default for
  cv::Matx33f cameraMatrix(525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0);

  std::vector<float> distortions;
  std::vector<int> inliers;

  //solvePnP(objectPoints, imagePoints, cameraMatrix, distortions, rvec, tvec, false, CV_EPNP);
  
  int initMinInliers = 7;
  int count = 0;

  while(inliers.size()==0 && count < 5)
  {
    inliers.clear();
    solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distortions, rvec, tvec, false, 
          100, //iterations 
          10, // reproj error 
          initMinInliers, // min inliers 
          inliers, CV_EPNP);
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

  q = R3_rotation_to_quaternion(rot_mat_r3);

  euler = euler_angle(rotmat);


  // todo: check ordering
  std::cout << "roll" << "\t\t" << "pitch" << "\t\t" << "yaw" << std::endl;
  std::cout << euler[0] <<"\t" << euler[1] << "\t" << euler[2] << std::endl; 
  std::cout << tvec << std::endl;
  //std::cout << inliers.size() << std::endl;

  return inliers.size();
}



#endif