/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Raphael Favier, Technical University Eindhoven, (r.mysurname <aT> tue.nl)
 */


#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/texture_mapping.h>

#include <pcl/io/vtk_lib_io.h>

using namespace pcl;

/** \brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses */
void showCameras (pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

  // visualization object
  pcl::visualization::PCLVisualizer visu ("cameras");
  
  // add the mesh's cloud (colored on Z axis)
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloud, "z");
  visu.addPointCloud (cloud, color_handler, "cloud");

  //visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");


  // add a visual for each camera at the correct pose
  for(int i = 0 ; i < cams.size () ; ++i)
  {
    // read current camera
    pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
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
    ss << "Cam #" << i+1;
    visu.addText3D(ss.str (), p1, 0.1, 1.0, 1.0, 1.0, ss.str ());

    ss.str ("");
    ss << "camera_" << i << "line1";
    visu.addLine (p1, p2,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line2";
    visu.addLine (p1, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line3";
    visu.addLine (p1, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line4";
    visu.addLine (p1, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line5";
    visu.addLine (p2, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line6";
    visu.addLine (p5, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line7";
    visu.addLine (p4, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line8";
    visu.addLine (p3, p2,ss.str ());
  }

  // add a coordinate system
  visu.addCoordinateSystem (1.0);

  // reset camera
  visu.resetCamera ();
  
  // wait for user input
  visu.spin ();
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
bool readCamPoseFile(std::string filename, pcl::TextureMapping<pcl::PointXYZ>::Camera &cam)
{
  ifstream myReadFile;
  myReadFile.open(filename.c_str (), ios::in);
  if(!myReadFile.is_open ())
  {
    PCL_ERROR ("Error opening file %d\n", filename.c_str ());
    return false;
  }
  myReadFile.seekg(ios::beg);

  char current_line[1024];
  double val;
  
  // go to line 2 to read translations
  GotoLine(myReadFile, 2);
  myReadFile >> val; cam.pose (0,3)=val; //TX
  myReadFile >> val; cam.pose (1,3)=val; //TY
  myReadFile >> val; cam.pose (2,3)=val; //TZ

  // go to line 7 to read rotations
  GotoLine(myReadFile, 7);

  myReadFile >> val; cam.pose (0,0)=val;
  myReadFile >> val; cam.pose (0,1)=val;
  myReadFile >> val; cam.pose (0,2)=val;

  myReadFile >> val; cam.pose (1,0)=val;
  myReadFile >> val; cam.pose (1,1)=val;
  myReadFile >> val; cam.pose (1,2)=val;

  myReadFile >> val; cam.pose (2,0)=val;
  myReadFile >> val; cam.pose (2,1)=val;
  myReadFile >> val; cam.pose (2,2)=val;

  cam.pose (3,0) = 0.0;
  cam.pose (3,1) = 0.0;
  cam.pose (3,2) = 0.0;
  cam.pose (3,3) = 1.0; //Scale
  
  // go to line 12 to read camera focal length and size
  GotoLine (myReadFile, 12);
  myReadFile >> val; cam.focal_length=val; 
  myReadFile >> val; cam.height=val;
  myReadFile >> val; cam.width=val;  
  
  // close file
  myReadFile.close ();

  return true;

}

int main (int argc, char** argv)
{

  // Prepare point cloud data structure
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Load up the map pointcloud
  PCL_INFO ("\nLoading Map Point Cloud %s...\n", argv[1]);
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    // Return failure to load
    PCL_ERROR("Could not load point cloud %s \n", argv[1]);
    return -1;
  }
  
  // Shrink the pointcloud
  for(size_t i =0; i < cloud->size (); ++i)
  {
    cloud->points[i].x*=0.006;
    cloud->points[i].y*=0.006;
    cloud->points[i].z*=0.006;

  }
  // Load textures and cameras poses and intrinsics
  PCL_INFO ("\nLoading textures and camera poses...\n");

  pcl::texture_mapping::CameraVector my_cams;
  
  const boost::filesystem::path base_dir (".");
  std::string extension (".txt");
  int cpt_cam = 0;
  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if(boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
      readCamPoseFile(it->path ().string (), cam);
      cam.texture_file = boost::filesystem::basename (it->path ()) + ".png";
      my_cams.push_back (cam);
      cpt_cam++ ;
    }
  }


  PCL_INFO ("\tLoaded %d textures.\n", my_cams.size ());
  PCL_INFO ("...Done.\n");
  
  // Display cameras to user
  //(PCL_INFO ("\nDisplaying cameras. Press \'q\' to continue texture mapping\n");
    showCameras(my_cams, cloud);
  
    cin.get();
    cin.get();
  

  return (0);
}
