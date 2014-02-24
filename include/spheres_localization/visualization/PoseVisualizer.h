#ifndef SL_VISU_POSEVISU_H
#define SL_VISU_POSEVISU_H

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

class PoseVisualizer 
{
public:
	PoseVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneCloud,
					pcl::PointCloud<pcl::PointXYZ>::Ptr interestCloud);
	void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void);
	void updateCorrespondences(const spheres_localization::point_match_array& correspondences);
	void updateMatches(const spheres_localization::point_match_array& matches);
	void updateImage(const cv::Mat &img);
	void updatePose(const pcl::TextureMapping<pcl::PointXYZ>::Camera cam);
	void spinOnce();
	bool ok(){return escPressed;}

private:
	pcl::visualization::PCLVisualizer::Ptr visu;

	bool escPressed,
		paused,
		cameraTrack;
	bool shouldDrawMatches,
		shouldDrawCorresp;
	int lastMatchCount,
		lastCorrespCount;

	Eigen::Affine3f lastPose;

	void clearLines(std::string name, int count);
	void drawLines(const spheres_localization::point_match_array& points, 
		std::string name, 
		float red, 
		float green, 
		float blue);

};

#endif