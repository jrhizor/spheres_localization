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

namespace enc = sensor_msgs::image_encodings;

// TODO: Fix global badness, maybe a singleton
pcl::visualization::PCLVisualizer visu ("cameras");
cv_bridge::CvImagePtr rgb;
cv_bridge::CvImage rgbI;

int lastMatchCount = 0;

bool new_image;
bool cameraTrack = true;
Eigen::Affine3f lastPose;

class InvalidFileException : public std::exception
{
public:
   InvalidFileException(char* filename) { m_filename = filename; }

   char* m_filename;
};

/**
 * Function: redrawCameras
 * Input: PCL file containing camera position data (refer to PCL documentation)
 * Output: Redraws the camera frustrum in the visualizer window
 **/
void redrawCameras (pcl::TextureMapping<pcl::PointXYZ>::Camera cam)
{
  // read current camera
  double focal = cam.focal_length;
  double height = cam.height;
  double width = cam.width;
  
  // create a 5-point visual for each camera
  pcl::PointXYZ p1, p2, p3, p4, p5;
  p1.x=0; p1.y=0; p1.z=0;
  double angleX = RAD2DEG (2.0 * atan (width / (2.0*focal)));
  double angleY = RAD2DEG (2.0 * atan (height / (2.0*focal)));
  double dist = 0.75;
  double minX, minY, maxX, maxY;
  maxX = dist*tan (atan (width / (2.0*focal)));
  minX = -maxX;
  maxY = dist*tan (atan (height / (2.0*focal)));
  minY = -maxY;
  p2.x=minX; p2.y=minY; p2.z=dist;
  p3.x=maxX; p3.y=minY; p3.z=dist;
  p4.x=maxX; p4.y=maxY; p4.z=dist;
  p5.x=minX; p5.y=maxY; p5.z=dist;
  p1=pcl::transformPoint (p1, cam.pose);
  p2=pcl::transformPoint (p2, cam.pose);
  p3=pcl::transformPoint (p3, cam.pose);
  p4=pcl::transformPoint (p4, cam.pose);
  p5=pcl::transformPoint (p5, cam.pose);
  std::stringstream ss;
  ss << "Cam";
  visu.removeShape(ss.str());
  visu.addText3D(ss.str (), p1, 0.1, 1.0, 1.0, 1.0, ss.str ());
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

  ss.str ("");
  ss << "camera" << "line1";
  visu.removeShape(ss.str());
  visu.addLine (p1, p2,ss.str ());
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());
  
  ss.str ("");
  ss << "camera" << "line2";
  visu.removeShape(ss.str());
  visu.addLine (p1, p3,ss.str ());
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

  ss.str ("");
  ss << "camera" << "line3";
  visu.removeShape(ss.str());
  visu.addLine (p1, p4,ss.str ());
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

  ss.str ("");
  ss << "camera" << "line4";
  visu.removeShape(ss.str());
  visu.addLine (p1, p5,ss.str ());
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());
  
  ss.str ("");
  ss << "camera" << "line5";
  visu.removeShape(ss.str());
  visu.addLine (p2, p5,ss.str ());
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());
  
  ss.str ("");
  ss << "camera" << "line6";
  visu.removeShape(ss.str());
  visu.addLine (p5, p4,ss.str ());
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());
  
  ss.str ("");
  ss << "camera" << "line7";
  visu.removeShape(ss.str());
  visu.addLine (p4, p3,ss.str ());
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());
  
  ss.str ("");
  ss << "camera" << "line8";
  visu.removeShape(ss.str());
  visu.addLine (p3, p2,ss.str ());
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

}

/**
 * Function: readCamPoseStreamMsg
 * Input: A String message from the ros data stream
 *        Camera position data (Refer to PCL documentation)
 * Output: Camera position data
 *         A boolean indicating if the data was valid
 **/
// TODO: Rename this function to something more succinct 
bool readCamPoseStreamMsg(const spheres_localization::pose& msg, pcl::TextureMapping<pcl::PointXYZ>::Camera &cam)
{
  // Translation Information
  cam.pose (0,3)=msg.x; //TX
  cam.pose (1,3)=msg.y; //TY
  cam.pose (2,3)=msg.z; //TZ

  // Rotation Matrix
  cam.pose (0,0)=msg.rot_mat[0];
  cam.pose (0,1)=msg.rot_mat[1];
  cam.pose (0,2)=msg.rot_mat[2];

  cam.pose (1,0)=msg.rot_mat[3];
  cam.pose (1,1)=msg.rot_mat[4];
  cam.pose (1,2)=msg.rot_mat[5];

  cam.pose (2,0)=msg.rot_mat[6];
  cam.pose (2,1)=msg.rot_mat[7];
  cam.pose (2,2)=msg.rot_mat[8];

  // Scale
  cam.pose (3,0) = 0.0;
  cam.pose (3,1) = 0.0;
  cam.pose (3,2) = 0.0;
  cam.pose (3,3) = 1.0; //Scale
  
  // Hard coded (to kinect values) since the phone will not have this information available
  cam.focal_length = 575.816f;
  cam.height = 480;
  cam.width = 640;

  // Check that the data is valid, return false if failed
  if(cam.pose(0,3) == 0 && cam.pose(1,3) == 0 && cam.pose(2,3) == 0 )
  {
    return false;
  }

  return true;

}

/**
 * Function: poseEstimateCallback
 * Input: A special pose message from the ros data stream
 * Output: Processes the data from the ros stream and updates the visualization
 **/
void poseEstimateCallback(const spheres_localization::pose& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  pcl::TextureMapping<pcl::PointXYZ>::Camera cam;

  if (readCamPoseStreamMsg(msg, cam)) 
  {
    redrawCameras(cam);
  }

}

/**
 * Function: imageHandleCallback
 * Input: A special camera message from the ros data stream
 * Output: Processes the data from the ros stream and updates the visualization
 **/
void imageHandleCallback(const sensor_msgs::ImageConstPtr& msg)
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

  float factor = 800.0f;

  // Prepare new image for rendering in the visualizer
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr imageCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr imageCloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

  cv::Mat img = rgbI.image;


  //float factor = 800.0f;
  // Camera offset: (0.1f,0.1f,2.5f)

  // TODO: Allow the image to be posted stationary
  // TODO: Make the image get bigger when you make it stationary
  // TODO: Post the controls on the screen, as well as enable an escape button

  for(size_t i = 0; i < img.size().height; i++)
  { 
    for(size_t j = 0; j < img.size().width; j++)
    {
      pcl::PointXYZRGB tempPoint;
      tempPoint.x = ((float)-j)/factor + 0.1f;
      tempPoint.y = ((float)-i)/factor + 0.1f;
      tempPoint.z = 2.5f;
      tempPoint.r = img.at<cv::Vec3b>( i, j )[2];
      tempPoint.g = img.at<cv::Vec3b>( i, j )[1];
      tempPoint.b = img.at<cv::Vec3b>( i, j )[0];

      imageCloud->push_back(tempPoint);
    }
  }

  // Remove the old imagecloud from the visualizer and add the new one
  if(cameraTrack)
  {
    pcl::transformPointCloud (*imageCloud, *imageCloud2 , visu.getViewerPose());
    lastPose = visu.getViewerPose();
  }
  else
  {
    pcl::transformPointCloud (*imageCloud, *imageCloud2, lastPose);        
  }
  
  visu.removePointCloud("imageCloud");
  visu.addPointCloud(imageCloud2,"imageCloud");

}

/**
 * Function: correspondenceCallback
 * Input: A special point correspondence message from the ros data stream
 * Output: Processes the data from the ros stream and updates the visualization
 **/
void correspondenceCallback(const spheres_localization::point_match_array& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  // Store the correspondences locally
  // float factor = 800.0f;
  // Camera offset: (0.1f,0.1f,2.5f)
  std::stringstream ss;
  pcl::PointXYZ p1, p2;
  float factor = 800.0f;

  for(size_t i=0; i < lastMatchCount; i++)
  {
    ss << "MatchLine" << i;
    visu.removeShape(ss.str());   
    ss.str (""); 
  }


  for(size_t i=0; i < msg.matches.size(); i++)
  {
    p1.x = -msg.matches[i].u/factor +0.1f;
    p1.y = -msg.matches[i].v/factor +0.1f;
    p1.z = 2.48f;

    p2.x = msg.matches[i].x;
    p2.y = msg.matches[i].y;
    p2.z = msg.matches[i].z;

    ss << "MatchLine" << i;
    visu.addLine (pcl::transformPoint (p1, lastPose), p2,ss.str ());
    visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, ss.str());
    ss.str ("");
  }

  lastMatchCount = msg.matches.size();
}

/**
 * Function: loadSceneCloud
 * Input: A Filename for the scene's pointcloud
 * Output: The scene is loaded into the visualizer
 * Throws: InvalidFileException
 **/
void loadSceneCloud(char* sceneFilename) 
{
  // Prepare point cloud data structure
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);

  // Load up the map pointcloud
  PCL_INFO ("\nLoading Map Point Cloud %s...\n", sceneFilename);
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (sceneFilename, *cloud) == -1)
  {
    // Return failure to load
    PCL_ERROR("Could not load point cloud %s \n", sceneFilename);
    throw InvalidFileException(sceneFilename);
  }
  
  // Shrink the pointcloud to the same scale as the camera data (smaller distance between points)
  for(size_t i =0; i < cloud->size (); ++i)
  {
    cloud->points[i].x*=0.006;
    cloud->points[i].y*=0.006;
    cloud->points[i].z*=0.006;

  }

  // Downsample the pointcloud (less points in the cloud)
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f,0.01f,0.01f);
  sor.filter(*cloudFiltered);

  // Add the mesh's cloud (colored on X axis)
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloudFiltered, "z");
  visu.addPointCloud (cloudFiltered, color_handler, "cloud");
}

/**
 * Function: loadInterestMap
 * Input: A Filename for the interest point map
 * Output: The interest points are loaded into the visualizer
 * Throws: InvalidFileException
 **/
void loadInterestMap(char* mapFilename)
{
  // Initialize the pseudorandom number generator
  srand (time(NULL));

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



  // Add the map point cloud with a random color
  int red = rand() % 255, green=rand() % 255, blue=rand() % 255;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler2 (mapCloud,red, green, blue);
  std::stringstream cloudName;
  cloudName << "mapCloud";
  visu.addPointCloud(mapCloud, color_handler2, cloudName.str() );
  visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudName.str());  
}

/**
 * Function: redrawImageOverlay
 * Input: The camera image and the list of correspondences
 * Output: The image is redrawn onto the visualizer along with the correspondences
 * Throws: InvalidFileException
 **/
void redrawImageOverlay()
{
  // Define Function Variables

  // Remove the old image pointcloud

  // Remove the old correspondences

  // Convert the image into a pointcloud

  // Transform the image by the camera position and orientation

  // Offset the image to a good position

  // Draw the correspondences

}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "p" && event.keyDown ())
  {
    std::cout << "p pressed, stopping tracking" << std::endl;

    cameraTrack = !cameraTrack;
  }
}

int main (int argc, char** argv)
{

  // Fail and die if incorrect number of arguments was given
  if( argc != 4 )
  {
    std::cout << "ERROR: Incorrect number of arguments given" << std::endl
              << "./application mapPointCloud interestPointFile cameraRosTopic" << std::endl;
    return -1;
  }

  // Prepare ros for listening to the data stream
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  // Pose Estimation handling
  ros::Subscriber subPose = n.subscribe("pose_estimation", 1000, poseEstimateCallback);

  // Image Handling
  image_transport::ImageTransport it(n);
  image_transport::Subscriber rgb_sub;
  rgb_sub = it.subscribe(argv[3], 1, imageHandleCallback);

  // Correspondence handling
  ros::Subscriber subCorresp = n.subscribe("point_match_array", 1000, correspondenceCallback);

  // Keyboard Listener
  visu.registerKeyboardCallback (keyboardEventOccurred, (void*)&visu);


  // Do all of the map loading
  try
  {
    // Load the scene cloud
    loadSceneCloud(argv[1]);

    // Load the interest points
    loadInterestMap(argv[2]);
  }
  catch(InvalidFileException e)
  {
    std::cout << "ERROR: File: " << e.m_filename << " does not appear to be valid." << std::endl;   
    return -1;
  }

  // Finish initializing the visualizer
  visu.addCoordinateSystem (1.0);
  visu.resetCamera ();
  

  // Spin through ros callbacks and pcl window monitoring
  while(ros::ok())
  {
    // Do the spinning
    visu.spinOnce();
    ros::spinOnce();

    // Redraw the image and the point correspondences

  }

  return (0);
}
