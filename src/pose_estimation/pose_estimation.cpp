#include <spheres_localization/pose_estimation/pose_estimation.h>


int main(int argc, char **argv)
{
  // rosrun spheres_localization pose_estimation /home/jrhizor/Desktop/KinFuSnapshots/map.txt SIFT topicname
  ROS_ASSERT(argc==4);
  std::string map_file_name = argv[1];
  std::string method = argv[2];
  std::string camera_topic = argv[3];

  // load map
  std::vector<InterestPoint3D> world_map = load_map(map_file_name);
  ROS_INFO("Loaded map with %d points.", int(world_map.size()));

  // handle map
  std::vector<cv::KeyPoint> map_keypoints;
  cv::Mat map_desc(world_map.size(), world_map[0].descriptor.size(), CV_32F);
  PtLookupTable map_position_lookup;
  ROS_INFO("Created containers for map information.");

  for(unsigned int i=0; i<world_map.size(); ++i)
  {
    cv::KeyPoint temp_keypoint;
    temp_keypoint.pt = cv::Point2f(world_map[i].x, world_map[i].y);

    for(unsigned int j=0; j<world_map[i].descriptor.size(); ++j)
    {
      map_desc.at<float>(i,j) = world_map[i].descriptor[j];
    }

    map_position_lookup[std::make_pair(temp_keypoint.pt.x, temp_keypoint.pt.y)] = 
          pcl::PointXYZ(world_map[i].x,world_map[i].y,world_map[i].z);

    map_keypoints.push_back(temp_keypoint);
  }

  ROS_INFO("Filled containers with map information.");




  // initialize stuff
  int timeDetect, timeDescribe, timeMatch, timePE;
  int totalDetect=0, totalDescribe=0, totalMatch=0, totalInliers=0, totalGoodMatches=0, totalPE = 0,
      numGoodMatches=0, numInliers=0;
  int numQueries = 12;



  cv::Mat tvec;
  ::boost::math::quaternion<double> q;


  std::string queryFile = "/home/jrhizor/Desktop/KinFuSnapshots/0.png";

  cv::Mat img = cv::imread(queryFile, 0); 

  std::cout << img.size().height << " " <<  img.size().width << std::endl;

  for(int i=0; i<1; ++i)
  {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat desc;

    getFeatures(method, img, keypoints, desc, timeDetect, timeDescribe);

    findMatchesAndPose(map_desc, desc, map_keypoints, keypoints, numInliers, numGoodMatches, 
              timeMatch, timePE, img, tvec, q, map_position_lookup);

    // output pose 
    std::cout << "tvec " << tvec << std::endl;
    std::cout  << " " <<
          tvec.at<double>(0,0) << " " << tvec.at<double>(0,1) << " " << tvec.at<double>(0,2) << " " <<
          q.R_component_1() <<" "<< q.R_component_2()  << " " << q.R_component_3() << " " << q.R_component_4() << 
          std::endl;

    std::cout << std::endl << std::endl;
  }

  return 0;
} 



  // ORB
  // SIFT
  // SURF
  // FAST+FREAK
  // MSER+ORB
  // MSER+Moments

  // output overall processing time, detection, description, matching times, error from ground truth, number of points detected, number of points retained
