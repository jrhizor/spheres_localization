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

// TODO: Get rid of extra includes

// Holder over from pose
namespace enc = sensor_msgs::image_encodings;

MessageHandler::MessageHandler(PoseVisualizer *visu, std::string cameraStream)
{
	// Get the visualizer
	this->visualizer = visu;

	// Init IT
	it = new image_transport::ImageTransport(n);

	// Pose Estimation handling
	subPose = n.subscribe("pose_estimation", 1, &MessageHandler::callbackReadPoseMessage, this);

	// Image Handling
	rgb_sub = it->subscribe(cameraStream.c_str(), 1, &MessageHandler::callbackReadImageMessage, this);

	// Correspondence handling
	subCorresp = n.subscribe("point_match_array", 1, &MessageHandler::callbackReadCorrespMessage, this);	

	// Match handling
	subMatches = n.subscribe("point_match_array_all", 1, &MessageHandler::callbackReadMatchesMessage, this);		
}

void MessageHandler::callbackReadPoseMessage(const spheres_localization::pose& msg)
{
	// Initialize function variables
	pcl::TextureMapping<pcl::PointXYZ>::Camera cam;

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
	cam.pose (3,3) = 1.0; 

	// Hard coded (to kinect values) since the phone will not have this information available
	// TODO: Get this information from the phone
	cam.focal_length = 575.816f;
	cam.height = 480;
	cam.width = 640;

	// If data is valid, pass it to the visualizer
	if(cam.pose(0,3) != 0 || cam.pose(1,3) != 0 || cam.pose(2,3) != 0 )
	{
		visualizer->updatePose(cam);
	}

}

void MessageHandler::callbackReadImageMessage(const sensor_msgs::ImageConstPtr& msg)
{
	// Function variables
	cv_bridge::CvImagePtr rgb;

	// Try to decode the image into something useful
	try
	{
		cv_bridge::CvImagePtr rgb = cv_bridge::toCvCopy(msg, enc::BGR8);

		// Tell the visualizer to update its image
		visualizer->updateImage(rgb->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

}

void MessageHandler::callbackReadCorrespMessage(const spheres_localization::point_match_array& corresp)
{
	visualizer->updateCorrespondences(corresp);
}

void MessageHandler::callbackReadMatchesMessage(const spheres_localization::point_match_array& matches)
{
	visualizer->updateMatches(matches);
}