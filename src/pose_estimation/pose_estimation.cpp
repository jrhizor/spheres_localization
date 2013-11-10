#include <spheres_localization/pose_estimation/pose_estimation.h>

int main(int argc, char **argv) // rosrun spheres_localization pose_estimation /home/jrhizor/Desktop/KinFuSnapshots/map.txt SIFT /camera/image_raw
{
  ros::init(argc, argv, "pose_estimation");

  // rosrun spheres_localization pose_estimation /home/jrhizor/Desktop/KinFuSnapshots/map.txt SIFT topicname
  ROS_ASSERT(argc==4);
  std::string map_file_name = argv[1];
  std::string method = argv[2];
  std::string camera_topic = argv[3]; 

  //  export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"

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


  PoseEstimator estimator(camera_topic);
  estimator.run(map_keypoints, map_desc, map_position_lookup, method);

  return 0;
} 



  // ORB
  // SIFT
  // SURF
  // FAST+FREAK
  // MSER+ORB
  // MSER+Moments

  // output overall processing time, detection, description, matching times, error from ground truth, number of points detected, number of points retained
