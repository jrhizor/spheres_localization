Spheres Localization
====================

## Charting
	rosrun spheres_localization input_folder num_images descriptor_type descriptor_size output_file_path maxdescdist 

## Pose Estimation
	rosrun spheres_localization pose_estimation map_file descriptor_type camera_stream

## Visualization
	rosrun spheres_localization visualization world_point_cloud map_file camera_stream image_directory

	NOTE: Last parameter of visualization may be omitted for a precolored pointcloud

To use a bag file for visualization:

	rosbag play bag_file -l -r 0.1

## Dependencies
* ROS Hydro
* PCL
* OpenCV
* Boost

## Visualization Example for Garage Dataset
	roscore

	rosbag play ~/data/garage/left3ft.bag -l -r 0.1

	rosrun spheres_localization pose_estimation ~/data/garage/KinFuSnapshots/map.txt SIFT /camera/rgb/image_color

	rosrun spheres_localization visualization ~/data/garage/world_color.pcd ~/data/garage/KinFuSnapshots/map.txt /camera/rgb/image_color

