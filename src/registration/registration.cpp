/* registration.cpp
 * Author: Alex McArther
 * Last Edited: 10/9/13
 * 
 * Main file for the registration component of spheres localization
 *
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

bool loadPointClouds(char* filepath, std::vector<PointCloud::Ptr> cloudSet)
{
	// Define function variables
	std::ifstream fin;
	std::string directory = std::string(filepath);
	std::string fname = directory + "info.txt";
	int filecount;


	// Open the information file
	fin.open(fname.c_str());
	if(!fin.good())
	{
		return false;
	}
	fin >> filecount;
	fin.close();

	// Load all of the pointclouds
	for(int index=0; index < filecount; index++)
	{
		PointCloud::Ptr tempCloud (new PointCloud);

		// Get the pointcloud's name
		std::stringstream ss;
		ss << directory << "/depth/" << index+1 << ".pcd";

		// Read pointcloud
		pcl::io::loadPCDFile(ss.str(), *tempCloud);

		// Remove NAN values from pointcloud
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*tempCloud,*tempCloud, indices);

		// Add that pointcloud to our list
		cloudSet.push_back(tempCloud);
	}

	return true;

}

int main(int argc, char* argv[]) 
{
	// Define function variables
	std::ifstream fin;
	std::ofstream fout;
	int imageIndex = -1, totalImages;
	PointCloud::Ptr totalCloud (new PointCloud);
	PointCloud::Ptr newCloud (new PointCloud);
	PointCloud resultCloud;
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	pcl::PassThrough<PointT> pass;
	std::vector<PointCloud::Ptr> cloudSet;
	std::string folderPath;

	std::stringstream ss;

	bool fileCorrelated;
	char pcdFileName[] = "capture";
	char totalPath[700];
	char errorString[700];
	char resultPath[700];

	// Verify command line arguments
	if(argc < 2)
	{
		// Tell the user to give us a file next time
		std::cout << "Please specify filepath." << std::endl;

		// Return failure
		return -1;
	}

	// Read in the pointclouds from files
	loadPointClouds(argv[1], cloudSet);
	folderPath = argv[1];

	// If count is less than 2, return failure
	if(cloudSet.size() < 2)
	{
		PCL_ERROR ("Error reading files. \n");
		return -1;
	}
	PCL_INFO("Loaded %d pointclouds.", (int)cloudSet.size());

	// TODO: Identity matrix for mat 1

	// Copy pointcloud 1 into aggregated point cloud
	*totalCloud = *cloudSet[1];

	// Process all pointclouds
	ss << folderPath << "transforms.txt";
	fout.open(ss.str().c_str());
	for(int index=1; index < cloudSet.size(); index++)
	{
		// Align the pointclouds		
		icp.setInputSource(cloudSet[index]);
		icp.setInputTarget(totalCloud);
		icp.align(resultCloud);

		if( icp.hasConverged() )
		{
			// Add the transformed pointcloud to the aggregate
			*totalCloud += resultCloud;

			// Output the transformation
			fout << icp.getFinalTransformation() << std::endl;
		}
	}
	fout.close();

	// Save the aggregated pointcloud in the root directory
	pcl::io::savePCDFile ("totalCloud.pcd", *totalCloud);

	// Return successful completion
	return 0;
}
