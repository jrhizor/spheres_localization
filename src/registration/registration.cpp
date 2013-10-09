/* registration.cpp
 * Author: Alex McArther
 * Last Edited: 10/9/13
 * 
 * Main file for the registration component of spheres localization
 *
 */

#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int main(int argc, char* argv[]) 
{
	// Define function variables
	std::ifstream fin;
	std::ofstream fout;
	int imageCount = -1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr totalCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> resultCloud;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	int folderIndex = 0;
	bool fileCorrelated;
	char pcdFileName[] = "capture.pcd";
	char *folderPath;
	char *totalPath;
	char *errorString;
	char *resultPath;

	// Verify command line arguments
	if(argc < 2)
	{
		// Tell the user to give us a file next time
		std::cout << "Please specify filename." << std::endl;

		// Return failure
		return 1;
	}

	// Open file specified in command line
	fin.open(argv[1]);

	// Verify that file is open
	if(!fin.good())
	{
		// Tell the user that he/she gave us a bad file
		std::cout << "File not found." << std::endl;

		// Return failure
		return 1;
	}

	// Get the number of image/point cloud pairs
	fin >> imageCount;

	// Close file
	fin.close();

	// If count is less than 2, return failure
	if(imageCount < 2)
	{
		std::cout << "Operation invalid on less than two files." << std::endl;
		return -1;
	}

	// Read in first pointcloud into aggregate and save identity matrix there
	if(pcl::io::loadPCDFile<pcl::PointXYZ> ("/0/capture.pcd", *totalCloud) == -1)
	{
		PCL_ERROR ("Could not load file /0/capture.pcd \n");
		return -1;
	}
	// TODO: Identity matrix

	// Prime the loop
	fileCorrelated = true;
	imageCount--;
	folderIndex++;

	// While files remain, and no files are invalid, add pointclouds
	while( imageCount > 0 && fileCorrelated == true)
	{
		// Disable correlation flag
		fileCorrelated = false;

		// Determine file path
		sprintf(folderPath, "/%i",folderIndex);
		sprintf(totalPath, "/%i/%s",folderIndex,pcdFileName);

		// Read in new pointcloud
		if(pcl::io::loadPCDFile<pcl::PointXYZ> (totalPath, *newCloud) == -1)
		{
			sprintf(errorString, "Could not load file %s .\n", totalPath);
			PCL_ERROR (errorString);
			return -1;
		}		

		// Run correlation from new cloud to aggregate
		icp.setInputSource(newCloud);
		icp.setInputTarget(totalCloud);
		icp.align(resultCloud);

		// If correlation was successful, continue through
		if( icp.hasConverged() )
		{
			// Transform the input matrix

			// Add the input matrix to the total matrix
			*totalCloud = resultCloud;

			// Save transform matrix to same directory as cloud
			sprintf(resultPath, "%s/transform.txt", folderPath);
			fout.open(resultPath);
			fout << icp.getFinalTransformation();
			fout.close();

			// Let the loop continue
			fileCorrelated = true;

		}

	}

	// Save the aggregated pointcloud in the root directory
	pcl::io::savePCDFile ("totalCloud.pcd", *totalCloud);

	// Return successful completion
	return 0;
}
