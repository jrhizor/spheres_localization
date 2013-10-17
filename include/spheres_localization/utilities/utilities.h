#ifndef SL_UTILITIES_H
#define SL_UTILITIES_H

#include <stdint.h>

#include <string>
#include <sstream>


#include <cassert>
#include <functional>

#include <stdio.h>
#include <time.h>
#include <ctime>
#include <vector>
#include <cmath>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <pcl/io/oni_grabber.h>
#include <pcl/io/image_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>

#include <boost/shared_array.hpp>
#include <boost/math/quaternion.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

pcl::PointXYZ get_3d_point(const cv::Mat &depth, int u, int v)
{
	// uses Microsoft Kinect constants
	float fx = 525.0;
	float fy = 525.0;
	float cx = 319.5;
	float cy = 239.5;

	float factor = 5000; 

	float X, Y, Z;

	Z = depth.at<int16_t>(v,u) / factor;
	X = (u - cx) * Z / fx;
	Y = (v - cy) * Z / fy;

	return pcl::PointXYZ(X,Y,Z);
}

// TODO: convert to newer OpenCV standards
bool euler_angle(const CvMat& rot, CvPoint3D64f& euler) {
	double _R[9], _Q[9];
	CvMat R, Q;
	CvMat *pQx=NULL, *pQy=NULL, *pQz=NULL;  // optional. For debugging.
	cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);
	cvInitMatHeader(&Q,  3, 3, CV_64FC1, _Q);

	cvRQDecomp3x3(&rot, &R, &Q, pQx, pQy, pQz, &euler);
	return true;
}

#endif