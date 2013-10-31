#ifndef SL_REGISTERED_MAPS_H
#define SL_REGISTERED_MAPS_H

struct RegImg
{
  std::string rgb_file;
  std::string depth_file;
  cv::Mat translation, rotation;
};

std::vector<RegImg> load_sequence(const std::string &input_folder, int num_images)
{
  std::vector<RegImg> reg_imgs;

  for(int i=0; i<num_images; ++i)
  {
    std::cout << "\tProcessing frame " << i << std::endl;

    RegImg frame;

    // record file names
    frame.rgb_file = input_folder + boost::lexical_cast<std::string>(i) + std::string(".png");
    frame.depth_file = input_folder + boost::lexical_cast<std::string>(i) + std::string("_d.png");
    std::string transform_file = input_folder + boost::lexical_cast<std::string>(i) + std::string(".txt");

    std::cout << "\t\tRGB File" << frame.rgb_file << std::endl;
    std::cout << "\t\tDepth File" << frame.depth_file << std::endl;
    std::cout << "\t\tTransform File" << transform_file << std::endl;

    // load transformation
    std::ifstream fin(transform_file.c_str());

    cv::Mat rotation(3, 3, CV_32FC1);
    cv::Mat translation(3, 1, CV_32FC1);

    std::string temp;

    fin >> temp;

    fin >> translation.at<float>(0, 0) 
        >> translation.at<float>(1, 0) 
        >> translation.at<float>(2, 0);

    fin >> temp;

    fin >> rotation.at<float>(0, 0)
        >> rotation.at<float>(0, 1)
        >> rotation.at<float>(0, 2)
        >> rotation.at<float>(1, 0)
        >> rotation.at<float>(1, 1)
        >> rotation.at<float>(1, 2)
        >> rotation.at<float>(2, 0)
        >> rotation.at<float>(2, 1)
        >> rotation.at<float>(2, 2);

    fin.close();

    frame.translation = translation;
    frame.rotation = rotation;

    reg_imgs.push_back(frame);
  }

  return reg_imgs;
}


std::vector<std::vector<double> > generate_3d_desc(const RegImg &img, const std::string &type, int type_size)
{
  cv::Mat rgb = cv::imread(img.rgb_file, CV_LOAD_IMAGE_COLOR);
  cv::Mat depth = cv::imread(img.depth_file, CV_LOAD_IMAGE_GRAYSCALE);

  ROS_ASSERT(rgb.rows != 0 && rgb.cols != 0 && depth.rows != 0 && depth.cols != 0);

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat desc;

  if(type.compare(std::string("ORB"))==0)
  {
    cv::ORB orbDect(700, 1.2f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31); 
    orbDect.detect(rgb, keypoints);
    orbDect.compute(rgb, keypoints, desc);
  }
  else if(type.compare(std::string("SIFT"))==0)
  {
    cv::SiftFeatureDetector siftDect; 
    siftDect.detect(rgb, keypoints);
    siftDect.compute(rgb, keypoints, desc);
  }
  else if(type.compare(std::string("SURF"))==0)
  {
    cv::SURF surfDect(100, 5,1,true, false);
    surfDect(rgb, cv::noArray(), keypoints, desc);
  }
  else if(type.compare(std::string("FAST+FREAK"))==0)
  {
    cv::FREAK freakDect;  
    cv::FAST(rgb, keypoints, 20/*threshold*/);
    freakDect.compute(rgb, keypoints, desc);
  }
  else if(type.compare(std::string("MSER+ORB"))==0)
  {
    cv::Ptr<cv::FeatureDetector> detector = new cv::MserFeatureDetector();
    cv::ORB orbDect(700, 1.2f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31); 
    detector->detect(rgb, keypoints);
    orbDect.compute(rgb, keypoints, desc);
  } 
  else if(type.compare(std::string("FAST+ORB"))==0)
  {
    cv::ORB orbDect(700, 1.2f, 5, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31); 
    cv::FAST(rgb, keypoints, 20/*threshold*/);
    orbDect.compute(rgb, keypoints, desc);
  }
  else
  {
    ROS_ERROR("The descriptor-detector pair type is invalid.");
    exit(-1);
  }

  std::vector<std::vector<double> > map_elements;

  for(unsigned int i=0; i<keypoints.size(); ++i)
  {
    // TODO: check if u and v are the right order
    int u = keypoints[i].pt.x;
    int v = keypoints[i].pt.y;

    pcl::PointXYZ pos = get_3d_point(depth, u, v);
    cv::Mat mat_pos(3, 1, CV_32FC1);

    // TODO: check if this has to be in homogenous coordinates
    mat_pos.at<float>(0,0) = pos.x;
    mat_pos.at<float>(1,0) = pos.y;
    mat_pos.at<float>(2,0) = pos.z;

    cv::Mat trans_pos = img.rotation * (mat_pos+img.translation);

    std::vector<double> map_element;

    map_element.push_back(trans_pos.at<float>(0,0));
    map_element.push_back(trans_pos.at<float>(1,0));
    map_element.push_back(trans_pos.at<float>(2,0));

    // TODO: check if this is grabbing the correct part of desc
    for(unsigned int j=0; j<type_size; ++j)
    {
      map_element.push_back(desc.at<float>(i, j));
    }

    map_elements.push_back(map_element);
  }

  std::cout << "\t\tDescriptors: " << map_elements.size() << std::endl;

  return map_elements;
}

void generate_map(const std::vector<RegImg> &reg_imgs, const std::string &output_file, 
                  const std::string &type, int type_size)
{
  std::ofstream fout(output_file.c_str());

  fout << reg_imgs.size() << std::endl
       << type_size << std::endl;

  fout.precision(15);

  for(unsigned int i=0; i<reg_imgs.size(); ++i)
  {
    std::cout << "\tProcessing frame " << i << std::endl;
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

#endif