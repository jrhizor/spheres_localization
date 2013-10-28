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

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>

#include <boost/shared_array.hpp>
#include <boost/math/quaternion.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <spheres_localization/utilities/rottoquat.h>

/*
using namespace std;
using namespace cv;
using namespace pcl;

int x;

void createCloud(const Mat &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	float fx = 525.0;
  float fy = 525.0;
	float cx = 319.5;
	float cy = 239.5;

	float factor = 5000; 

	float X, Y, Z;

	for(int v=0; v<depth.rows; v++)
	{
		for(int u=0; u<depth.cols; u++)
		{
			X = (u - cx) * Z / fx;
		  Y = (v - cy) * Z / fy;
	   	Z = depth.at<float>(v,u) / factor;

	   	// push 3d point
	   	cloud->push_back(PointXYZ(X,Y,Z));
		}
	}   
}

bool eulerAngle(const CvMat& rot, CvPoint3D64f& euler) {
	double _R[9], _Q[9];
	CvMat R, Q;
	CvMat *pQx=NULL, *pQy=NULL, *pQz=NULL;  // optional. For debugging.
	cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);
	cvInitMatHeader(&Q,  3, 3, CV_64FC1, _Q);

	cvRQDecomp3x3(&rot, &R, &Q, pQx, pQy, pQz, &euler);
	return true;
}

PointXYZ get3DPoint(const Mat &depth, int u, int v)
{
	float fx = 525.0;
	float fy = 525.0;
	float cx = 319.5;
	float cy = 239.5;

	float factor = 5000; 

	float X, Y, Z;

	Z = depth.at<int16_t>(v,u) / factor;
	X = (u - cx) * Z / fx;
	Y = (v - cy) * Z / fy;

	// push 3d point
	//cout << X << " " << Y << " " << Z << " --- " << depth.at<int16_t>(v,u) << " " << u << " " << v<< endl;

	return PointXYZ(X,Y,Z);
}

void getFeatures(const string &method, const Mat &img, vector<KeyPoint> &keypoints, Mat &desc, int &timeDetect, 
						int &timeDescribe)
{
	if(method.compare(string("ORB"))==0)
	{
		ORB orbDect(700, 1.2f, 5, 31, 0, 2, ORB::HARRIS_SCORE, 31);	
		int startMark = clock();
		orbDect.detect(img, keypoints);
		int detectMark = clock();
		orbDect.compute(img, keypoints, desc);
		int describeMark = clock();

		// record times
		timeDetect = detectMark - startMark;
		timeDescribe = describeMark - detectMark;
	}
	else if(method.compare(string("SIFT"))==0)
	{
		keypoints.clear();
		desc.release();

		SiftFeatureDetector siftDect;	
		int startMark = clock();
		siftDect.detect(img, keypoints);
		int detectMark = clock();
		siftDect.compute(img, keypoints, desc);
		int describeMark = clock();

		// record times
		timeDetect = detectMark - startMark;
		timeDescribe = describeMark - detectMark;
	}
	else if(method.compare(string("SURF"))==0)
	{
		keypoints.clear();
		desc.release();

		SURF surfDect(100, 5,1,true, false);
		int startMark = clock();
		surfDect(img, cv::noArray(), keypoints, desc);
		int describeMark = clock();

		// record times
		timeDetect =  (describeMark - startMark)/2.0;
		timeDescribe = (describeMark - startMark)/2.0;
	}
	else if(method.compare(string("FAST+FREAK"))==0)
	{
		FREAK freakDect;	
		int startMark = clock();
		FAST(img, keypoints, 20//threshold
		);
		int detectMark = clock();
		freakDect.compute(img, keypoints, desc);
		int describeMark = clock();

		// record times
		timeDetect = detectMark - startMark;
		timeDescribe = describeMark - detectMark;
	}
	else if(method.compare(string("MSER+ORB"))==0)
	{
		Ptr<FeatureDetector> detector = new MserFeatureDetector();
		ORB orbDect(700, 1.2f, 5, 31, 0, 2, ORB::HARRIS_SCORE, 31);	

		int startMark = clock();
		detector->detect( img, keypoints );
		int detectMark = clock();
		orbDect.compute(img, keypoints, desc);
		int describeMark = clock();

		// record times
		timeDetect = detectMark - startMark;
		timeDescribe = describeMark - detectMark;
	}	
	else if(method.compare(string("FAST+ORB"))==0)
	{
		ORB orbDect(700, 1.2f, 5, 31, 0, 2, ORB::HARRIS_SCORE, 31);	
		int startMark = clock();
		FAST(img, keypoints, 20 //threshold
			);
		int detectMark = clock();
		orbDect.compute(img, keypoints, desc);
		int describeMark = clock();

		// record times
		timeDetect = detectMark - startMark;
		timeDescribe = describeMark - detectMark;
	}
	else
	{
		cout << "NO VALID METHOD SELECTED" << endl;
		exit(-1);
	}
}


void findMatchesAndPose(Mat &desc, Mat &desc2, const vector<KeyPoint> &keypoints, const vector<KeyPoint> &keypoints2, 
						pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Mat &depth, int &numInliers, 
						int &numGoodMatches, int &timeMatch, int &timePE, const Mat &mapImg, const Mat &queryImg,
						Mat &tvec, ::boost::math::quaternion<double> &q);

int pnp(const vector<KeyPoint> &keypoints, const vector<KeyPoint> &keypoints2, 
					  const vector<DMatch> &good_matches, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Mat &depth, 
					  Mat &tvec, ::boost::math::quaternion<double> &q);


int main()
{
	// initialize stuff
  	string method = "FAST+ORB";
  	int timeDetect, timeDescribe, timeMatch, timePE;
  	int totalDetect=0, totalDescribe=0, totalMatch=0, totalInliers=0, totalGoodMatches=0, totalPE = 0,
  		numGoodMatches=0, numInliers=0;
  	int numQueries = 12;
  	string query_path = "/home/jared/Desktop/spheres_localization/kinect_report/rgbd_dataset_freiburg1_desk/rgb/";
  	const char *queryImg_src[] = {"1305031454.391677.png", "1305031454.427465.png",
							   	  "1305031454.459913.png", "1305031454.491617.png", "1305031454.527700.png",
							   	  "1305031454.559575.png", "1305031454.591635.png", "1305031454.627580.png",
							   	  "1305031454.659528.png", "1305031454.691884.png", "1305031454.727659.png",
							   	  "1305031454.759732.png"};
	vector<string> queryImg(queryImg_src, queryImg_src+numQueries);



	Mat tvec;
	::boost::math::quaternion<double> q;

  	cout << "Method: " << method << endl;

  	ofstream fout;
  	stringstream ss;
  	ss << "/home/jared/Desktop/spheres_localization/kinect_report/" << method << "_acc.txt";
  	fout.open(ss.str().c_str());

	// get map
    string mapFileRGB = "/home/jared/Desktop/spheres_localization/kinect_report/map/frame_0_rgb.png"; // rgb/1305031454.791641.png";
    Mat mapRGB = imread(mapFileRGB, 0); 
    string mapFileDepth ="/home/jared/Desktop/spheres_localization/kinect_report/map/frame_0_depth.png"; //depth/1305031453.359684.png";
    Mat mapDepth = imread(mapFileDepth, 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    createCloud(mapDepth, cloud);

    vector<KeyPoint> map_keypoints;
    Mat map_desc;

    getFeatures(method, mapRGB, map_keypoints, map_desc, timeDetect, timeDescribe);
    cout << "Map keypoints: " << map_keypoints.size() << endl; 

	// for each image in sequence, estimate the pose
    for(int i=0; i<numQueries; ++i)
    {
    	cout << "Query Image #" << i << endl;
    	string queryFile = query_path + queryImg[i];
    	Mat img = imread(queryFile, 0); 
    	vector<KeyPoint> keypoints;
    	Mat desc;

    	cout << "image filename: " << queryFile << endl;

    	getFeatures(method, img, keypoints, desc, timeDetect, timeDescribe);
    	cout << "Detection time: " << 1.0*timeDetect/CLOCKS_PER_SEC << endl;
    	cout << "Description time: " << 1.0*timeDescribe/CLOCKS_PER_SEC << endl;
    	totalDetect += timeDetect;
    	totalDescribe += timeDescribe;

    	cout << keypoints.size() << " " << desc.size() << endl;

    	findMatchesAndPose(map_desc, desc, map_keypoints, keypoints, cloud, mapDepth, numInliers, numGoodMatches, 
    						timeMatch, timePE, mapRGB, img, tvec, q);
    	cout << "Matching time: " << 1.0*timeMatch/CLOCKS_PER_SEC << endl;
    	cout << "Pose Estimation time: " << 1.0*timePE/CLOCKS_PER_SEC << endl;
    	cout << "Num good matches: " << 1.0*numGoodMatches << endl;
    	cout << "Num inliers: " << 1.0*numInliers << endl;
    	totalMatch += timeMatch;
    	totalPE += timePE;
    	totalGoodMatches += numGoodMatches;
    	totalInliers += numInliers;

    	// output pose 
    	string timestamp = queryImg[i];
    	timestamp.resize(17);
    	cout << "tvec " << tvec << endl;
    	fout << timestamp << " " <<
    		    tvec.at<double>(0,0) << " " << tvec.at<double>(0,1) << " " << tvec.at<double>(0,2) << " " <<
    		    q.R_component_1() <<" "<< q.R_component_2()  << " " << q.R_component_3() << " " << q.R_component_4() << 
    		    endl;


    	cout << endl << endl;
    }

    // output statistics
    cout << "Total Detection time: " << 1.0*totalDetect/CLOCKS_PER_SEC << endl;
    cout << "Total Description time: " << 1.0*totalDescribe/CLOCKS_PER_SEC << endl;
    cout << "Total Matching time:" << 1.0*totalMatch/CLOCKS_PER_SEC << endl;
    cout << "Total PE time:" << 1.0*totalPE/CLOCKS_PER_SEC << endl;
    cout << "Total good matches:" << totalGoodMatches << endl;
    cout << "Total inliers:" << totalInliers << endl << endl;
    cout << "Total total time: " << 1.0*(totalDetect+totalDescribe+totalMatch+totalPE)/CLOCKS_PER_SEC << endl;

    cout << "Avg Detection time: " << (1.0*totalDetect/CLOCKS_PER_SEC)/numQueries << endl;
    cout << "Avg Description time: " << (1.0*totalDescribe/CLOCKS_PER_SEC)/numQueries << endl;
    cout << "Avg Matching time:" << (1.0*totalMatch/CLOCKS_PER_SEC)/numQueries << endl;
    cout << "Avg PE time:" << (1.0*totalPE/CLOCKS_PER_SEC)/numQueries << endl;
    cout << "Avg good matches:" << 1.0*totalGoodMatches/numQueries << endl;
    cout << "Avg inliers:" << 1.0*totalInliers/numQueries << endl;
    cout << "Avg total time: " << (1.0*(totalDetect+totalDescribe+totalMatch+totalPE)/CLOCKS_PER_SEC)/numQueries << endl;
    cout << "Avg fps: " << 1.0/((1.0*(totalDetect+totalDescribe+totalMatch+totalPE)/CLOCKS_PER_SEC)/numQueries) << endl;

    fout.close();

	return 0;
} 



	// ORB
	// SIFT
	// SURF
	// FAST+FREAK
	// MSER+ORB
	// MSER+Moments

	// output overall processing time, detection, description, matching times, error from ground truth, number of points detected, number of points retained

	// also need to look at Android to Kinect matches vs Kinect to Kinect matches

	// if time, use ICP to generate a larger map and test images over that.



void findMatchesAndPose(Mat &desc, Mat &desc2, const vector<KeyPoint> &keypoints, const vector<KeyPoint> &keypoints2, 
						pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Mat &depth, int &numInliers, 
						int &numGoodMatches, int &timeMatch, int &timePE, const Mat &mapImg, const Mat &queryImg,
						Mat &tvec, ::boost::math::quaternion<double> &q)
{
	BruteForceMatcher<L2<float> > matcher;
	vector<vector<DMatch> > matches;
	int startMark, endMark;

	if(desc.type()!=CV_32F) {
    	desc.convertTo(desc, CV_32F);
	}

	if(desc2.type()!=CV_32F) {
    	desc2.convertTo(desc2, CV_32F);
	}
  	
  	startMark = clock();

  	matcher.knnMatch(desc, desc2, matches, 2);

	double ratio = 0.75;
	std::vector< DMatch > good_matches;
	for(int i = 0; i < matches.size(); i++)
	{
  		if(matches[i].size() == 2 && 
  	  		(matches[i][0].distance / matches[i][1].distance)<ratio // && good_matches.size() <20
  	  		&&
  	  		keypoints2[matches[i][0].queryIdx].pt.y <480 &&
  	  		keypoints2[matches[i][0].queryIdx].pt.x <640 &&
  	  		keypoints2[matches[i][0].queryIdx].pt.y >=0 &&
  	  		keypoints2[matches[i][0].queryIdx].pt.x >=0)
 		{
			float x = (float) depth.at<int16_t>(int(keypoints2[matches[i][0].queryIdx].pt.y),int(keypoints2[matches[i][0].queryIdx].pt.x));
			if(!(x != x) || x == 0)
	 	   	{
	 	   		good_matches.push_back(matches[i][0]);
	 	   	}
 	 	}
	}

	if(matches.size()==0)
	{
		cout << "0 matches" <<endl;
		exit(-1);
	}

	endMark = clock();
	timeMatch = endMark - startMark;


Mat result;
	    drawMatches(mapImg, keypoints, queryImg, keypoints2, good_matches, result, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS  ); 
   stringstream ss;//create a stringstream
   ss << "/home/jared/Desktop/spheres_localization/kinect_report/results/" << clock() << ".jpg";//add number to the stream

   cout << ss.str() << endl;
	//namedWindow(ss.str(), CV_WINDOW_AUTOSIZE );// Create a window for display.
	imwrite(ss.str(), result);
//     imshow( ss.str(), result );   
// char  aksk;
// cin >> aksk;


	startMark = clock();
	cout << "BEFORE EPNP" << endl;
	numInliers = pnp(keypoints, keypoints2, good_matches, cloud, depth, tvec, q);
	endMark = clock();

	timePE = endMark - startMark;

	numGoodMatches = good_matches.size();

}



int pnp(const vector<KeyPoint> &keypoints, const vector<KeyPoint> &keypoints2, 
					  const vector<DMatch> &good_matches, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Mat &depth, 
					  Mat &tvec, ::boost::math::quaternion<double> &q)
{
	vector<Point3f> objectPoints;
	vector<Point2f> imagePoints;
	
	CvPoint3D64f euler;

  	for(int i = 0; i < good_matches.size(); i++)
	{
		pcl::PointXYZ pt = get3DPoint(depth,keypoints[(good_matches[i].queryIdx)].pt.x, 
										keypoints[(good_matches[i].queryIdx)].pt.y);

    	double Xw = pt.x, Yw = pt.y, Zw = pt.z, u, v;

		u = keypoints2[(good_matches[i].trainIdx)].pt.x;
		v = keypoints2[(good_matches[i].trainIdx)].pt.y;

	    if(!(Zw != Zw))
	    {
			objectPoints.push_back(Point3f(Xw, Yw, Zw));
			imagePoints.push_back(Point2f(u,v));
	    }
 	}

	Mat rvec, rotmat, jacobian;

	// default for
	Matx33f cameraMatrix(525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0);

	vector<float> distortions;
	vector<int> inliers;

	//solvePnP(objectPoints, imagePoints, cameraMatrix, distortions, rvec, tvec, false, CV_EPNP);
	
	
	int initMinInliers = 7;
	int count = 0;

	while(inliers.size()==0 && count < 5)
	{
		inliers.clear();
		solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distortions, rvec, tvec, false, 
					100, //iterations 
					10, // reproj error 
					initMinInliers, // min inliers 
					inliers, CV_EPNP);
		if(initMinInliers<imagePoints.size()*.75) initMinInliers++;
		count++;
	}
	

	Rodrigues(rvec, rotmat, jacobian);

	// convert rotmat into quaternion
	R3_matrix<double> rot_mat_r3;
	rot_mat_r3.a11 = rotmat.at<double>(0,0);
	rot_mat_r3.a12 = rotmat.at<double>(0,1);
	rot_mat_r3.a13 = rotmat.at<double>(0,2);
	rot_mat_r3.a21 = rotmat.at<double>(1,0);
	rot_mat_r3.a22 = rotmat.at<double>(1,1);
	rot_mat_r3.a23 = rotmat.at<double>(1,2);
	rot_mat_r3.a31 = rotmat.at<double>(2,0);
	rot_mat_r3.a32 = rotmat.at<double>(2,1);
	rot_mat_r3.a33 = rotmat.at<double>(2,2);

	q = R3_rotation_to_quaternion(rot_mat_r3);

	eulerAngle(rotmat, euler);


	cout << "roll" << "\t\t" << "pitch" << "\t\t" << "yaw" << endl;
	cout << euler.z <<"\t" << euler.x << "\t" << euler.y << endl;	
	cout << tvec << endl;
	//cout << inliers.size() << endl;

	return inliers.size();
}
*/

void load_map(const std::string &input)
{
  std::ofstream fout(output_file.c_str());

  fout << reg_imgs.size() << std::endl
       << type_size << std::endl;

  fout.precision(15);

  for(unsigned int i=0; i<reg_imgs.size(); ++i)
  {
    std::vector<std::vector<double> > output_lines = generate_3d_desc(reg_imgs[i], type, type_size);
    
    for(unsigned int j=0; j<output_lines.size(); ++j)
    {
      ROS_ASSERT(output_lines[j].size()-3==type_size);

      for(unsigned int k=0; k<3; ++k)
      {
        fout << output_lines[j][k] << " ";
      }

      fout << std::endl;

      for(unsigned int k=3; k<output_lines[j].size(); ++k)
      {
        fout << output_lines[j][k] << " ";
      }

      fout << std::endl;
    }
  }

  fout.close();
}

int main(int argc, char const *argv[])
{
	
	return 0;
}

