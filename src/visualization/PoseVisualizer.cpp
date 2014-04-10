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


// TODO: Get rid of extra includes

PoseVisualizer::PoseVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneCloud,
					pcl::PointCloud<pcl::PointXYZ>::Ptr interestCloud)
{
	// Initialize the flags
	escPressed = false;
	paused = false;
	cameraTrack = true;
	displayMatches = false;
	displayCorresp = true;
	lastMatchCount = 0;
	lastCorrespCount = 0;

	// Init Visualizer
	visu = new pcl::visualization::PCLVisualizer("Pose Estimation Visualizer");

	// Prime the image pointcloud, for performance (use updatePointCloud instead of remove & add)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	visu->addPointCloud(cloud, "ImageCloud");

	// Print the information text
	visu->addText("Spacebar: Pause Visualization", 15, 100, "v1 text");
	visu->addText("L: Lock the 2d image in place", 15, 80, "v2 text");
	visu->addText("Esc: Close the Visualizer", 15, 60, "v3 text");
	visu->addText("M: Toggle display of the matches", 15, 40, "v4 text");
	visu->addText("C: Toggle display of the correspondences", 15, 20, "v5 text");

	// Add the point clouds to the visualizer
	visu->addPointCloud(sceneCloud, "SceneCloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> interestColor(interestCloud, 255, 0, 255);
	visu->addPointCloud<pcl::PointXYZ> (interestCloud, interestColor, "InterestCloud");
	visu->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "InterestCloud");

	// Keyboard Listener
	visu->registerKeyboardCallback (&PoseVisualizer::keyboardEventOccurred, *this, (void*)visu);

	// Finish initializing the visualizer
	visu->addCoordinateSystem (1.0);
	visu->resetCamera ();

	// TEMP: Initialize camera to a good position for Jared
	visu->setCameraPosition(-0.763717, 1.30741, -4.50887, 0, -1,-1);

}


void PoseVisualizer::clearLines(std::string name, int count)
{
	// Function variables
	std::stringstream ss;

    // Remove old lines
    for(size_t i=0; i < count; i++)
    {
      ss << name << i;
      visu->removeShape(ss.str());   
      ss.str (""); 
    }	
}

void PoseVisualizer::drawLines(const spheres_localization::point_match_array& points, 
	std::string name, 
	float red, 
	float green, 
	float blue)
{
	// Function variables
	std::stringstream ss;
	pcl::PointXYZ p1, p2;
	float imageScaleFactor = 800.0f;

    // Draw the match lines from the pose estimator
    for(size_t i=0; i < points.matches.size(); i++)
    {
      // NOTE: Z is calibrated to a distance from the screen (backward)
      p1.x = -points.matches[i].u/imageScaleFactor +0.1f;
      p1.y = -points.matches[i].v/imageScaleFactor +0.1f;
      p1.z = 2.48f;

      p2.x = points.matches[i].x;
      p2.y = points.matches[i].y;
      p2.z = points.matches[i].z;

      ss << name << i;
      visu->addLine (pcl::transformPoint (p1, lastPose), p2,ss.str ());
      visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, red, green, blue, ss.str());
      ss.str ("");
    }
}

void PoseVisualizer::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "l" && event.keyDown ())
  {
    std::cout << "KEY EVENT: l pressed, Toggling picture lock." << std::endl;

    cameraTrack = !cameraTrack;
  }
  else if (event.getKeySym() == "Escape" && event.keyDown())
  {
    std::cout << "KEY EVENT: ESC pressed, closing." << std::endl;
    escPressed = true;
  }
  else if (event.getKeySym() == "space" && event.keyDown())
  {
    std::cout << "KEY EVENT: Space pressed, pausing visualization" << std::endl;
    paused = !paused;
  }
  else if (event.getKeySym() == "m" && event.keyDown())
  {
  	std::cout << "KEY EVENT: m pressed, Toggling matches" << std::endl;
  	displayMatches = !displayMatches; 
  }
  else if (event.getKeySym() == "c" && event.keyDown())
  {
  	std::cout << "KEY EVENT: c pressed, Toggling correspondences" << std::endl;
  	displayCorresp = !displayCorresp;
  }
}

void PoseVisualizer::updateCorrespondences(const spheres_localization::point_match_array& correspondences)
{
  // When not paused and we want, draw correspondences
  if (!paused && displayCorresp)
  {
    // Remove old corresp lines
    clearLines("CorrespLine", lastCorrespCount);

    // Draw the corresp lines from the pose estimator
   	drawLines(correspondences, "CorrespLine", 1.0f, 1.0f, 0.0f);

    // Store corresp count for removal next frame
    lastCorrespCount = correspondences.matches.size();
  }
  else if (!paused && lastCorrespCount > 0)
  {
    // Remove old corresp lines
    clearLines("CorrespLine", lastCorrespCount);
    lastCorrespCount = 0;  	
  }
}

void PoseVisualizer::updateMatches(const spheres_localization::point_match_array& matches)
{
  // When not paused and we want, draw matches
  if (!paused && displayMatches)
  {
    // Remove old match lines
    clearLines("MatchLine", lastMatchCount);

    // Draw the match lines from the pose estimator
   	drawLines(matches, "MatchLine", 0.0f, 1.0f, 1.0f);

    // Store match count for removal next frame
    lastMatchCount = matches.matches.size();
  }
  else if (!paused && lastMatchCount > 0)
  {
    // Remove old match lines
    clearLines("MatchLine", lastMatchCount);
    lastMatchCount = 0;
  }
}

void PoseVisualizer::updateImage(const cv::Mat &img)
{
	if(!paused)
	{
		// Scaling Factor
		const float factor = 800.0f;

		// Prepare new image for rendering in the visualizer
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr imageCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr imageCloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

		// Convert the image into a flat pointcloud
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

		// Move the cloud in front of the camera (if enabled)
		if(cameraTrack)
		{
		  pcl::transformPointCloud (*imageCloud, *imageCloud , visu->getViewerPose());
		  lastPose = visu->getViewerPose();
		}
		else
		{
		  pcl::transformPointCloud (*imageCloud, *imageCloud, lastPose);        
		}

		// Update the point cloud
		visu->updatePointCloud(imageCloud,"ImageCloud");
	}
}

void PoseVisualizer::updatePose(pcl::TextureMapping<pcl::PointXYZ>::Camera cam)
{
	if(!paused)
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
		visu->removeShape(ss.str());
		visu->addText3D(ss.str (), p1, 0.1, 1.0, 1.0, 1.0, ss.str ());
		visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

		ss.str ("");
		ss << "camera" << "line1";
		visu->removeShape(ss.str());
		visu->addLine (p1, p2,ss.str ());
		visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

		ss.str ("");
		ss << "camera" << "line2";
		visu->removeShape(ss.str());
		visu->addLine (p1, p3,ss.str ());
		visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

		ss.str ("");
		ss << "camera" << "line3";
		visu->removeShape(ss.str());
		visu->addLine (p1, p4,ss.str ());
		visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

		ss.str ("");
		ss << "camera" << "line4";
		visu->removeShape(ss.str());
		visu->addLine (p1, p5,ss.str ());
		visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

		ss.str ("");
		ss << "camera" << "line5";
		visu->removeShape(ss.str());
		visu->addLine (p2, p5,ss.str ());
		visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

		ss.str ("");
		ss << "camera" << "line6";
		visu->removeShape(ss.str());
		visu->addLine (p5, p4,ss.str ());
		visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

		ss.str ("");
		ss << "camera" << "line7";
		visu->removeShape(ss.str());
		visu->addLine (p4, p3,ss.str ());
		visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());

		ss.str ("");
		ss << "camera" << "line8";
		visu->removeShape(ss.str());
		visu->addLine (p3, p2,ss.str ());
		visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, ss.str());
	}
}

void PoseVisualizer::spinOnce()
{
	visu->spinOnce();
}
