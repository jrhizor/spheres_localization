#include <spheres_localization/pose_estimation/pose_estimation.h>

/*


int main()
{
  // initialize stuff
    string method = "FAST+ORB";
    int timeDetect, timeDescribe, timeMatch, timePE;
    int totalDetect=0, totalDescribe=0, totalcv::Match=0, totalInliers=0, totalGoodcv::Matches=0, totalPE = 0,
      numGoodMatches=0, numInliers=0;
    int numQueries = 12;
    string query_path = "/home/jared/Desktop/spheres_localization/kinect_report/rgbd_dataset_freiburg1_desk/rgb/";
    const char *queryImg_src[] = {"1305031454.391677.png", "1305031454.427465.png",
                    "1305031454.459913.png", "1305031454.491617.png", "1305031454.527700.png",
                    "1305031454.559575.png", "1305031454.591635.png", "1305031454.627580.png",
                    "1305031454.659528.png", "1305031454.691884.png", "1305031454.727659.png",
                    "1305031454.759732.png"};
  std::vector<string> queryImg(queryImg_src, queryImg_src+numQueries);



  cv::Mat tvec;
  ::boost::math::quaternion<double> q;

    std::cout << "Method: " << method << std::endl;

    ofstream fout;
    stringstream ss;
    ss << "/home/jared/Desktop/spheres_localization/kinect_report/" << method << "_acc.txt";
    fout.open(ss.str().c_str());

  // get map
    string mapFileRGB = "/home/jared/Desktop/spheres_localization/kinect_report/map/frame_0_rgb.png"; // rgb/1305031454.791641.png";
    cv::Mat mapRGB = imread(mapFileRGB, 0); 
    string mapFileDepth ="/home/jared/Desktop/spheres_localization/kinect_report/map/frame_0_depth.png"; //depth/1305031453.359684.png";
    cv::Mat mapDepth = imread(mapFileDepth, 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    createCloud(mapDepth, cloud);

    std::vector<cv::KeyPoint> map_keypoints;
    cv::Mat map_desc;

    getFeatures(method, mapRGB, map_keypoints, map_desc, timeDetect, timeDescribe);
    std::cout << "Map keypoints: " << map_keypoints.size() << std::endl; 

  // for each image in sequence, estimate the pose
    for(int i=0; i<numQueries; ++i)
    {
      std::cout << "Query Image #" << i << std::endl;
      string queryFile = query_path + queryImg[i];
      cv::Mat img = imread(queryFile, 0); 
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat desc;

      std::cout << "image filename: " << queryFile << std::endl;

      getFeatures(method, img, keypoints, desc, timeDetect, timeDescribe);
      std::cout << "Detection time: " << 1.0*timeDetect/CLOCKS_PER_SEC << std::endl;
      std::cout << "Description time: " << 1.0*timeDescribe/CLOCKS_PER_SEC << std::endl;
      totalDetect += timeDetect;
      totalDescribe += timeDescribe;

      std::cout << keypoints.size() << " " << desc.size() << std::endl;

      findMatchesAndPose(map_desc, desc, map_keypoints, keypoints, cloud, mapDepth, numInliers, numGoodMatches, 
                timeMatch, timePE, mapRGB, img, tvec, q);
      std::cout << "cv::Matching time: " << 1.0*timeMatch/CLOCKS_PER_SEC << std::endl;
      std::cout << "Pose Estimation time: " << 1.0*timePE/CLOCKS_PER_SEC << std::endl;
      std::cout << "Num good matches: " << 1.0*numGoodMatches << std::endl;
      std::cout << "Num inliers: " << 1.0*numInliers << std::endl;
      totalcv::Match += timeMatch;
      totalPE += timePE;
      totalGoodcv::Matches += numGoodMatches;
      totalInliers += numInliers;

      // output pose 
      string timestamp = queryImg[i];
      timestamp.resize(17);
      std::cout << "tvec " << tvec << std::endl;
      fout << timestamp << " " <<
            tvec.at<double>(0,0) << " " << tvec.at<double>(0,1) << " " << tvec.at<double>(0,2) << " " <<
            q.R_component_1() <<" "<< q.R_component_2()  << " " << q.R_component_3() << " " << q.R_component_4() << 
            std::endl;


      std::cout << std::endl << std::endl;
    }

    // output statistics
    std::cout << "Total Detection time: " << 1.0*totalDetect/CLOCKS_PER_SEC << std::endl;
    std::cout << "Total Description time: " << 1.0*totalDescribe/CLOCKS_PER_SEC << std::endl;
    std::cout << "Total cv::Matching time:" << 1.0*totalcv::Match/CLOCKS_PER_SEC << std::endl;
    std::cout << "Total PE time:" << 1.0*totalPE/CLOCKS_PER_SEC << std::endl;
    std::cout << "Total good matches:" << totalGoodcv::Matches << std::endl;
    std::cout << "Total inliers:" << totalInliers << std::endl << std::endl;
    std::cout << "Total total time: " << 1.0*(totalDetect+totalDescribe+totalcv::Match+totalPE)/CLOCKS_PER_SEC << std::endl;

    std::cout << "Avg Detection time: " << (1.0*totalDetect/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg Description time: " << (1.0*totalDescribe/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg cv::Matching time:" << (1.0*totalcv::Match/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg PE time:" << (1.0*totalPE/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg good matches:" << 1.0*totalGoodcv::Matches/numQueries << std::endl;
    std::cout << "Avg inliers:" << 1.0*totalInliers/numQueries << std::endl;
    std::cout << "Avg total time: " << (1.0*(totalDetect+totalDescribe+totalcv::Match+totalPE)/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg fps: " << 1.0/((1.0*(totalDetect+totalDescribe+totalcv::Match+totalPE)/CLOCKS_PER_SEC)/numQueries) << std::endl;

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

*/



int main(int argc, char const *argv[])
{
  
  return 0;
}

