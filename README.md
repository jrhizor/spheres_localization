spheres_localization
====================

# Charting
	rosrun spheres_localization input_folder num_images descriptor_type descriptor_size output_file_path maxdescdist 

# Pose Estimation
	rosrun spheres_localization pose_estimation map_file descriptor_type camera_stream

# Visualization
	rosrun spheres_localization visualization world_point_cloud map_file camera_stream

To use a bag file for visualization:

	rosbag play bag_file -l -r 0.1

# Dependencies
* ROS Hydro
* PCL
* OpenCV
* Boost