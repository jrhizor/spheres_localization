#include <spheres_localization/utilities/registered_maps.h>

int main(int argc, char **argv) // input_folder num_images descriptor_type descriptor_size output_file_path
{
  ros::init(argc, argv, "charting");

  ROS_ASSERT(argc==6);

  std::string input_folder = argv[1]; 
  int num_images = atoi(argv[2]);
  std::string desc_type = argv[3]; // descriptor type name
  int desc_type_size = atoi(argv[4]); // descriptor type size
  std::string output_path = argv[5];

  // force trailing slash
  if(input_folder[input_folder.size()-1]!='/')
  {
    input_folder = input_folder + std::string("/");
  }

  std::cout << "Loading sequence." << std::endl;
  std::vector<RegImg> reg_imgs = load_sequence(input_folder, num_images);


  std::cout << "Generating map." << std::endl;
  generate_map(reg_imgs, output_path, desc_type, desc_type_size);

  std::cout << "Finished." << std::endl;

  return 0;
}
