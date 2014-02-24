#ifndef SL_VISU_MSGHANDLE_H
#define SL_VISU_MSGHANDLE_H

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


class MessageHandler
{
public:
	MessageHandler(PoseVisualizer *visu, std::string cameraStream);
	bool ok(){return true;}

private:
	ros::NodeHandle n;
	ros::Subscriber subPose, subCorresp, subMatches;
	image_transport::ImageTransport *it;
	image_transport::Subscriber rgb_sub;

	PoseVisualizer *visualizer;
	void callbackReadPoseMessage(const spheres_localization::pose& msg);
	void callbackReadImageMessage(const sensor_msgs::ImageConstPtr& msg);
	void callbackReadCorrespMessage(const spheres_localization::point_match_array& corresp);
	void callbackReadMatchesMessage(const spheres_localization::point_match_array& matches);

};

#endif