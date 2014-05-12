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

## Warning!!!!!
Make sure you have added the following to your .bashrc file!
* source /opt/ros/hydro/setup.bash
* source ~/catkin_ws/devel/setup.bash

## Video Broadcast Process with the Android Phone
* Install "IP Webcam" app for android (should already be on the lab phone)
* Use ROS GSCam to relay IP Webcam images to a ROS camera topic

## Visualization Example for Garage Dataset
	roscore

	rosbag play ~/data/garage/left3ft.bag -l -r 0.1

	rosrun spheres_localization pose_estimation ~/data/garage/KinFuSnapshots/map.txt SIFT /camera/rgb/image_color

	rosrun spheres_localization visualization ~/data/garage/world_color.pcd ~/data/garage/KinFuSnapshots/map.txt /camera/rgb/image_color

