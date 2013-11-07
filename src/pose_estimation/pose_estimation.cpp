#include <spheres_localization/pose_estimation/pose_estimation.h>

// load map and method from command line
// load rgb and depth image from primesense
// do everything in a ros spin loop
// do average calculations cumulatively
// rgb as input

// somewhere assert that the method has the correct size

int main(int argc, char const *argv[]) // map_file_path descriptor_type 
{
  ROS_ASSERT(argc==3);

  // initialize 
  std::string map_file = argv[1];
  std::string method = argv[2];

  std::vector<InterestPoint3D> map3D = load_map(map_file);
  std::cout << "Loaded map file: " << map_file << std::endl;
  std::cout << "Using method: " << method << std::endl;


  int timeDetect, timeDescribe, timeMatch, timePE;
  int totalDetect=0, totalDescribe=0, totalMatch=0, totalInliers=0, totalGoodMatches=0, totalPE = 0,
    numGoodMatches=0, numInliers=0;
  int numQueries = 12;
  std::string query_path = "/home/jared/Desktop/spheres_localization/kinect_report/rgbd_dataset_freiburg1_desk/rgb/";
  const char *queryImg_src[] = {"1305031454.391677.png", "1305031454.427465.png",
                  "1305031454.459913.png", "1305031454.491617.png", "1305031454.527700.png",
                  "1305031454.559575.png", "1305031454.591635.png", "1305031454.627580.png",
                  "1305031454.659528.png", "1305031454.691884.png", "1305031454.727659.png",
                  "1305031454.759732.png"};
  std::vector<std::string> queryImg(queryImg_src, queryImg_src+numQueries);



  cv::Mat tvec;
  ::boost::math::quaternion<double> q;

    std::cout << "Method: " << method << std::endl;

    std::ofstream fout;
    std::stringstream ss;
    ss << "/home/jared/Desktop/spheres_localization/kinect_report/" << method << "_acc.txt";
    fout.open(ss.str().c_str());

  // get map
    std::string mapFileRGB = "/home/jared/Desktop/spheres_localization/kinect_report/map/frame_0_rgb.png"; // rgb/1305031454.791641.png";
    cv::Mat mapRGB = cv::imread(mapFileRGB, 0); 
    std::string mapFileDepth ="/home/jared/Desktop/spheres_localization/kinect_report/map/frame_0_depth.png"; //depth/1305031453.359684.png";
    cv::Mat mapDepth = cv::imread(mapFileDepth, 2);

    std::vector<cv::KeyPoint> map_keypoints;
    cv::Mat map_desc;

    getFeatures(method, mapRGB, map_keypoints, map_desc, timeDetect, timeDescribe);
    std::cout << "Map keypoints: " << map_keypoints.size() << std::endl; 



  // for each image in sequence, estimate the pose
    for(int i=0; i<numQueries; ++i)
    {
      std::cout << "Query Image #" << i << std::endl;
      std::string queryFile = query_path + queryImg[i];
      cv::Mat img = cv::imread(queryFile, 0); 
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat desc;

      std::cout << "image filename: " << queryFile << std::endl;

      getFeatures(method, img, keypoints, desc, timeDetect, timeDescribe);
      std::cout << "Detection time: " << 1.0*timeDetect/CLOCKS_PER_SEC << std::endl;
      std::cout << "Description time: " << 1.0*timeDescribe/CLOCKS_PER_SEC << std::endl;
      totalDetect += timeDetect;
      totalDescribe += timeDescribe;

      std::cout << keypoints.size() << " " << desc.size() << std::endl;

      findMatchesAndPose(map_desc, desc, map_keypoints, keypoints, mapDepth, numInliers, numGoodMatches, 
                timeMatch, timePE, mapRGB, img, tvec, q);
      std::cout << "cv::Matching time: " << 1.0*timeMatch/CLOCKS_PER_SEC << std::endl;
      std::cout << "Pose Estimation time: " << 1.0*timePE/CLOCKS_PER_SEC << std::endl;
      std::cout << "Num good matches: " << 1.0*numGoodMatches << std::endl;
      std::cout << "Num inliers: " << 1.0*numInliers << std::endl;
      totalMatch += timeMatch;
      totalPE += timePE;
      totalGoodMatches += numGoodMatches;
      totalInliers += numInliers;

      // output pose 
      std::string timestamp = queryImg[i];
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
    std::cout << "Total cv::Matching time:" << 1.0*totalMatch/CLOCKS_PER_SEC << std::endl;
    std::cout << "Total PE time:" << 1.0*totalPE/CLOCKS_PER_SEC << std::endl;
    std::cout << "Total good matches:" << totalGoodMatches << std::endl;
    std::cout << "Total inliers:" << totalInliers << std::endl << std::endl;
    std::cout << "Total total time: " << 1.0*(totalDetect+totalDescribe+totalMatch+totalPE)/CLOCKS_PER_SEC << std::endl;

    std::cout << "Avg Detection time: " << (1.0*totalDetect/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg Description time: " << (1.0*totalDescribe/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg cv::Matching time:" << (1.0*totalMatch/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg PE time:" << (1.0*totalPE/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg good matches:" << 1.0*totalGoodMatches/numQueries << std::endl;
    std::cout << "Avg inliers:" << 1.0*totalInliers/numQueries << std::endl;
    std::cout << "Avg total time: " << (1.0*(totalDetect+totalDescribe+totalMatch+totalPE)/CLOCKS_PER_SEC)/numQueries << std::endl;
    std::cout << "Avg fps: " << 1.0/((1.0*(totalDetect+totalDescribe+totalMatch+totalPE)/CLOCKS_PER_SEC)/numQueries) << std::endl;

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
