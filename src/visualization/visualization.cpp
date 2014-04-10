#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/io/vtk_lib_io.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "ros/ros.h"
#include  <signal.h>

#include <cmath>
#include <exception>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

#include <spheres_localization/pose.h>
#include <spheres_localization/point_match.h>
#include <spheres_localization/point_match_array.h>

#include <spheres_localization/visualization/PoseVisualizer.h>
#include <spheres_localization/visualization/MessageHandler.h>

// Struct: Used for matching poses with images for coloring the point cloud
struct ImageWithPose
{
  cv::Mat img;
  Eigen::Affine3f pose;

};

// Exception: Thrown by file loading functions (for catching in main)
class InvalidFileException : public std::exception
{
public:
   InvalidFileException(char* filename) { m_filename = filename; }

   char* m_filename;
};

/**
 * Function: loadSceneCloud
 * Input: A Filename for the scene's pointcloud
 * Output: The scene is loaded into the visualizer
 * Throws: InvalidFileException
 **/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadSceneCloud(char* sceneFilename, char* imageDirectory) 
{
  // Prepare point cloud data structure
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Load up the map pointcloud
  PCL_INFO ("\nINFO: Loading Map Point Cloud %s...\n", sceneFilename);
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (sceneFilename, *cloud) == -1)
  {
    // Return failure to load
    PCL_ERROR("Could not load point cloud %s \n", sceneFilename);
    throw InvalidFileException(sceneFilename);
  }

  // If we did not get RGB values, go ahead and try to determine them from the images
  if (cloud->points[0].r == 0 & cloud->points[0].g == 0 & cloud->points[0].b == 0)
  {
    // Notify the user
    std::cout << "INFO: Shrinking and thinning uncolored pointcloud" << std::endl;

    // Shrink the pointcloud to the same scale as the camera data (smaller distance between points)
    for(size_t i =0; i < cloud->size (); ++i)
    {
      cloud->points[i].x*=0.006;
      cloud->points[i].y*=0.006;
      cloud->points[i].z*=0.006;
    }

    // Downsample the pointcloud (less points in the cloud)
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f,0.01f,0.01f);
    sor.filter(*cloudFiltered);

    // Notify the user
    std::cout << "INFO: Attempting to color the pointcloud using a provided directory." << std::endl;
    if(imageDirectory == 0)
    {
      std:cout << "WARN: No image directory provided!" << std::endl;
    }

    // Prepare imagewithpose vector
    std::vector<ImageWithPose> imageSet;

    // Prepare string stream
    std::stringstream imageSS, poseSS;
    imageSS << imageDirectory << "/0.png";
    poseSS << imageDirectory << "/0.txt";
    int counter = 0;

    // Prepare distance array (for selecting nearest image points to color with)
    float *shortestCamDist = new float [cloudFiltered->size()];

    // Prime the loop (open first pose file)
    ifstream poseFin(poseSS.str().c_str());
    std::cout << "INFO: Loading images [1...n].png and pose [1...n].txt..." << std::endl; 

    // For each image
    while(poseFin.good())
    {
      ImageWithPose frame;
      frame.img = cv::imread(imageSS.str().c_str(), CV_LOAD_IMAGE_COLOR);

      if(! frame.img.data )
      {
        std::cout <<  "ERROR: Could not open or find image " << counter << ".png" << std::endl ;
      }
      else
      {
        std::cout << "\tFound and loaded " << counter << ".png" << std::endl;
        // Initialize the pose
        frame.pose.setIdentity();

        // File processing variables
        Eigen::Matrix3f rotation;
        Eigen::Vector3f translation;
        std::string temp;

        // Load the pose information from the text file
        poseFin >> temp;
        poseFin >> translation.x()
            >> translation.y()
            >> translation.z();
        poseFin >> temp;
        poseFin >> rotation.coeffRef(0, 0)
            >> rotation.coeffRef(0, 1)
            >> rotation.coeffRef(0, 2)
            >> rotation.coeffRef(1, 0)
            >> rotation.coeffRef(1, 1)
            >> rotation.coeffRef(1, 2)
            >> rotation.coeffRef(2, 0)
            >> rotation.coeffRef(2, 1)
            >> rotation.coeffRef(2, 2);

        // Combine translation and rotation, and get inverse (for ease of computation)
        frame.pose.translation() = translation;
        frame.pose.rotate(rotation);
        frame.pose = frame.pose.inverse();

        // Save the image in the vector
        imageSet.push_back(frame);
      }

      // Clean and prep for the next
      poseSS.str("");
      imageSS.str("");
      counter++;
      poseSS << imageDirectory << "/" << counter << ".txt";
      imageSS << imageDirectory << "/" << counter << ".png";
      poseFin.close();
      poseFin.open(poseSS.str().c_str());
    }

    // Image constants
    const float focal_length = 575.816f,
                cx = 319.5,
                cy = 239.5;

    std::cout << "INFO: Coloring using loaded images." << std::endl;

    // Color all of the points
    for(size_t i =0; i < cloudFiltered->size(); ++i)
    {
      // Initialize the color, and set the camera distance to really far
      cloudFiltered->points[i].r = 0;
      cloudFiltered->points[i].g = 255;
      cloudFiltered->points[i].b = 0;
      shortestCamDist[i] = 100000.0f;

      // Update the progress bar (every 1000 iterations)
      if(i%1000 == 0)
      {
        int completion = (((float)i)/cloudFiltered->size())*100.0f;
        int numBars = 1 + (completion/5);
        std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b"
                  << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
        std::cout << "Coloring: [";

        for(size_t j = 0; j < numBars; j++) std::cout << "=";
        for(size_t j = 0; j < 20-numBars; j++) std::cout << "-";
        std::cout << "] [" << completion+1 << "\%]";

      }     

      // Look through all images for this point
      for(std::vector<ImageWithPose>::iterator it = imageSet.begin(); it != imageSet.end(); ++it)
      {
        // Transform the point-cloud point by the camera transformation
        pcl::PointXYZRGB transformedPoint = transformPoint(cloudFiltered->points[i], it->pose);

        // Transform this point into screen coordinates
        int u,v;
        u = ((transformedPoint.x * focal_length) / transformedPoint.z) + cx;
        v = ((transformedPoint.y * focal_length) / transformedPoint.z) + cy;

        // Find the distance to the camera
        float camDistanceToPoint = sqrt(
          transformedPoint.x*transformedPoint.x +
          transformedPoint.y*transformedPoint.y +
          transformedPoint.z*transformedPoint.z);


        // Color the point if on camera and closer to camera than previous best
        if(u >= 0 && u < 640 && v >= 0 && v < 480 &&
          camDistanceToPoint < shortestCamDist[i])
        {
          cloudFiltered->points[i].r = it->img.at<cv::Vec3b>( v, u )[2];
          cloudFiltered->points[i].g = it->img.at<cv::Vec3b>( v, u )[1];
          cloudFiltered->points[i].b = it->img.at<cv::Vec3b>( v, u )[0];
          shortestCamDist[i] = camDistanceToPoint;
        }
      }
    }    
    // Save the colored point cloud for the future
    // Here i assume the file has .pcd extension (and remove it temporarily)
    // TODO: parse the file forward to the period, no "guessing"
    std::string oldFilename(sceneFilename);
    oldFilename = oldFilename.substr(0,oldFilename.length() - 4);
    std::stringstream ss;
    ss << oldFilename << "_color.pcd";
    std::cout << "\nINFO: Saving colored pointcloud to " << ss.str() << "." << std::endl;
    pcl::io::savePCDFile(ss.str(), *cloudFiltered, true);

    // Dealloc the distance array
    delete [] shortestCamDist;

    // Return the colored cloud
    cloud = cloudFiltered;
  }
  else
  {
    // No filtering, just load the pre-colored cloud
    std::cout << "INFO: Found colored point cloud, drawing directly." << std::endl;
  }

  // Notify completion
  std::cout << "INFO: Done processing scene pointcloud." << std::endl;

  // Return result of loading pointcloud
  return cloud;
}

/**
 * Function: loadInterestMap
 * Input: A Filename for the interest point map
 * Output: The interest points are loaded into the visualizer
 * Throws: InvalidFileException
 **/
pcl::PointCloud<pcl::PointXYZ>::Ptr loadInterestMap(char* mapFilename)
{
  // Prepare to read the map file
  std::ifstream fin(mapFilename);
  float valCatcher;

  // Fail and close if map file is not valid
  if (!fin.good())
    throw InvalidFileException(mapFilename);

  // Prime the map file by discarding the first two values
  fin >> valCatcher >> valCatcher;

  // Get all of those points from the map, skip the SIFT features
  pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  while(fin.good())
  {

    float x, y, z;

    // Read the position of the keypoint
    fin >> x >> y >> z;

    // Read and discard the sift features
    for(int k=0; k < 128; k ++)
    {
      fin >> valCatcher;
    }

    // Plot point
    mapCloud->push_back(pcl::PointXYZ(x,y,z));
    
  }

  // Return the interest point cloud
  return mapCloud;
}


int main (int argc, char** argv)
{

  // Define program variables
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr interestCloud;
  PoseVisualizer *visualizer;
  MessageHandler *msgHandler;

  // Prepare ROS?
  ros::init(argc, argv, "listener");

  // Validate the program arguments
  if( argc != 4 && argc != 5 )
  {
    std::cout << "ERROR: Incorrect number of arguments given" << std::endl
              << "./application mapPointCloud interestPointFile cameraRosTopic imageDirectory" << std::endl;
    return -1;
  }  

  // Attempt to load the scene file and the interest point file
  try
  {
    // Load the scene cloud
    sceneCloud = loadSceneCloud(argv[1], argv[4]);

    // Load the interest points
    interestCloud = loadInterestMap(argv[2]);
  }
  catch(InvalidFileException e)
  {
    std::cout << "ERROR: File: " << e.m_filename << " does not appear to be valid." << std::endl;   
    return -1;
  }

  // Initialize the visualizer
  visualizer = new PoseVisualizer(sceneCloud, interestCloud);

  // Initialize the message handler
  msgHandler = new MessageHandler(visualizer, argv[3]);

  // Exec until Visualizer or MessageHandler die
  while(visualizer->ok() && msgHandler->ok())
  {
    // Do the spinning
    visualizer->spinOnce();
    ros::spinOnce();

  }

  std::cout << "INFO: Closing program" << std::endl;

  // Clean up
  ros::shutdown();

  // Delete pointers
  delete visualizer;
  delete msgHandler;

  // Return completion
  return 0;
}
