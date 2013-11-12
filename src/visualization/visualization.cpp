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
#include "std_msgs/String.h"
#include  <signal.h>

#include <cmath>


// Global badness
pcl::visualization::PCLVisualizer visu ("cameras");

void  INThandler(int sig)
{

     signal(sig, SIG_IGN);
     exit(0);

}

/** \brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses */
void showCameras (pcl::TextureMapping<pcl::PointXYZ>::Camera cam)
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

/** \brief Helper function that jump to a specific line of a text file */
std::ifstream& GotoLine(std::ifstream& file, unsigned int num)
{
  file.seekg (std::ios::beg);
  for(int i=0; i < num - 1; ++i)
  {
    file.ignore (std::numeric_limits<std::streamsize>::max (),'\n');
  }
  return (file);
}

/** \brief Helper function that reads a camera file outputed by Kinfu */
bool readCamPoseFile(const std_msgs::String::ConstPtr& msg, pcl::TextureMapping<pcl::PointXYZ>::Camera &cam)
{

  // Declare function variables
  double val;
  std::stringstream dataStream;

  // Initialize the stream to read from
  dataStream.str(msg->data);
  
  // Translation Information
  dataStream >> val; cam.pose (0,3)=val; //TX
  dataStream >> val; cam.pose (1,3)=val; //TY
  dataStream >> val; cam.pose (2,3)=val; //TZ

  // Rotation Matrix
  dataStream >> val; cam.pose (0,0)=val;
  dataStream >> val; cam.pose (0,1)=val;
  dataStream >> val; cam.pose (0,2)=val;

  dataStream >> val; cam.pose (1,0)=val;
  dataStream >> val; cam.pose (1,1)=val;
  dataStream >> val; cam.pose (1,2)=val;

  dataStream >> val; cam.pose (2,0)=val;
  dataStream >> val; cam.pose (2,1)=val;
  dataStream >> val; cam.pose (2,2)=val;

  // Scale
  cam.pose (3,0) = 0.0;
  cam.pose (3,1) = 0.0;
  cam.pose (3,2) = 0.0;
  cam.pose (3,3) = 1.0; //Scale
  
  // Hard coded (to kinect values) since the phone will not have this information available
  cam.focal_length = 575.816f;
  cam.height = 480;
  cam.width = 640;

  return true;

}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  pcl::TextureMapping<pcl::PointXYZ>::Camera cam;

  readCamPoseFile(msg, cam);
  showCameras(cam);
}

int main (int argc, char** argv)
{
  srand (time(NULL));

  // Prepare ros for listening
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("pose_estimation", 1000, chatterCallback);
 
  // Prepare point cloud data structure
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);
  // Prepare visualization object

  // Load up the map pointcloud
  PCL_INFO ("\nLoading Map Point Cloud %s...\n", argv[1]);
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    // Return failure to load
    PCL_ERROR("Could not load point cloud %s \n", argv[1]);
    return -1;
  }
  
  // Shrink the pointcloud to the same scale as the camera data
  for(size_t i =0; i < cloud->size (); ++i)
  {
    cloud->points[i].x*=0.006;
    cloud->points[i].y*=0.006;
    cloud->points[i].z*=0.006;

  }

  // Downsample the pointcloud
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f,0.01f,0.01f);
  sor.filter(*cloudFiltered);

  // add the mesh's cloud (colored on Z axis)
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloudFiltered, "x");
  //pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_handler (cloud);
  visu.addPointCloud (cloudFiltered, color_handler, "cloud");
  //visu.addPointCloud(cloudFiltered, "cloud");

  // Draw the damn map
  std::ifstream fin("map.txt");
  float valCatcher;

  // Read some junk we dont care about
  fin >> valCatcher >> valCatcher;


  // Get all of those points
  for(int i =0; i < atoi(argv[2]); i++)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZ>);

    int red = rand() % 255, green=rand() % 255, blue=rand() % 255;
    float fred = ((float) red)/255.0f, fgreen = ((float) green)/255.0f,fblue = ((float) blue)/255.0f;

    for(int j=0;j < atoi(argv[3+i]); j++)
    {
      float x, y, z;

      fin >> x >> y >> z;

      // Read in all those junk features
      for(int k=0; k < 128; k ++)
      {
        fin >> valCatcher;
      }

      // Plot point
      mapCloud->push_back(pcl::PointXYZ(x,y,z));
      

      if (j == 0)
      {
        std::stringstream ss2;
        ss2 << "Perspective " << i;
        visu.addText3D(ss2.str(), pcl::PointXYZ(x,y,z), 0.05, fred, fgreen, fblue, ss2.str ());
      }
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler2 (mapCloud,red, green, blue);
    std::stringstream cloudName;
    cloudName << "mapCloud" << i;
    visu.addPointCloud(mapCloud, color_handler2, cloudName.str() );
    visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudName.str());


  }

  // Set point properties
  //visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "mapCloud");

  // add a coordinate system
  visu.addCoordinateSystem (1.0);

  // reset camera
  visu.resetCamera ();
  
  // Display cameras to user
  //(PCL_INFO ("\nDisplaying cameras. Press \'q\' to continue texture mapping\n");
  //showCameras(my_cams, cloudFiltered, visu);
  signal(SIGINT, INThandler);

  
  // Spin through ros callbacks and pcl window monitoring
  while(true)
  {
    visu.spinOnce();
    ros::spinOnce();
  }

  return (0);
}
