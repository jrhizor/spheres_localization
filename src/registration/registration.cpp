/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier
 * further adaptation by Alexander McArther */

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <cmath>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

// convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables 
	//our visualizer
	pcl::visualization::PCLVisualizer *p;
	//its left and right viewports
	int vp_1, vp_2;

//convenient structure to handle our pointclouds



// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation ()
	{
	// Define the number of dimensions
	nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointNormalT &p, float * out) const
	{
	// < x, y, z, curvature >
	out[0] = p.x;
	out[1] = p.y;
	out[2] = p.z;
	out[3] = p.curvature;
	}
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
	p->removePointCloud ("vp1_target");
	p->removePointCloud ("vp1_source");

	PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
	p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

	PCL_INFO ("Press q to begin the registration.\n");
	p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
	p->removePointCloud ("source");
	p->removePointCloud ("target");


	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
	if (!tgt_color_handler.isCapable ())
		PCL_WARN ("Cannot create curvature color handler!");

	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
	if (!src_color_handler.isCapable ())
		PCL_WARN ("Cannot create curvature color handler!");


	p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

	p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
	* \param argc the number of arguments (pass from main ())
	* \param argv the actual command line arguments (pass from main ())
	* \param models the resultant vector of point cloud datasets
	*/
bool loadData (int argc, char **argv, std::vector<PointCloud::Ptr, Eigen::aligned_allocator<PointCloud::Ptr> > &models)
{

	// Define function variables
	std::ifstream fin;
	int filecount;
	std::string datasetPath;

	// Open the file indicated by the console argument if argc is correct, otherwise print error and exit
	if(argc == 2)
	{
		// Load crucial information
		fin.open(argv[1]);

		// Read file if valid, otherwise die
		if(fin.good())
		{
			// Assuming that contents are there if the file actually opened
			fin >> filecount >> datasetPath;	
			fin.close();
		}
		else
		{
			return false;
		}

	}
	else
	{
		std::cout << "Usage: program_name info_filepath.txt" << std::endl 
			<< "Where info_filepath.txt contains the total number of images and their folder." 
			<< std::endl;

		return false;
	}

	// Open and load all of the files
	for(int index = 1; index <= filecount; index++)
	{
		// Define loop variables
		std::stringstream ss;
		PointCloud::Ptr cloud (new PointCloud);
		std::vector<int> resultIndices;

		// Rig up the file name
		ss << datasetPath << "capture" << index << ".pcd";

		// Pull data into point cloud
		pcl::io::loadPCDFile(ss.str(), *cloud);

		// Filter out NAN points (align doesn't like them)
		pcl::removeNaNFromPointCloud(*cloud, *cloud, resultIndices);

		// Push onto vector for later
		models.push_back(cloud);

	}

	// Finished sucessfully
	return true;
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
	* \param cloud_src the source PointCloud
	* \param cloud_tgt the target PointCloud
	* \param output the resultant aligned source PointCloud
	* \param final_transform the resultant transform between source and target
	*/
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets

	PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample)
	{
		grid.setLeafSize (0.05, 0.05, 0.05);
		grid.setInputCloud (cloud_src);
		grid.filter (*src);

		grid.setInputCloud (cloud_tgt);
		grid.filter (*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}


	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (30);
	
	norm_est.setInputCloud (src);
	norm_est.compute (*points_with_normals_src);
	pcl::copyPointCloud (*src, *points_with_normals_src);

	norm_est.setInputCloud (tgt);
	norm_est.compute (*points_with_normals_tgt);
	pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon (1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (0.1);  
	// Set the point representation
	reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	reg.setInputSource (points_with_normals_src);
	reg.setInputTarget (points_with_normals_tgt);



	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations (2);
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO ("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource (points_with_normals_src);
		reg.align (*reg_result);

			//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

			//if the difference between this transformation and the previous one
			//is smaller than the threshold, refine the process by reducing
			//the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
		
		prev = reg.getLastIncrementalTransformation ();

		// visualize current state
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

	//p->removePointCloud ("source");
	//p->removePointCloud ("target");

	PCL_INFO ("Press q to continue the registration.\n");
	p->spin ();

	//p->removePointCloud ("source"); 
	//p->removePointCloud ("target");

	//add the source to the transformed target
	*output += *cloud_src;
	
	final_transform = targetToSource;
 }


/* ---[ */
int main (int argc, char** argv)
{
	// Define function variables
	std::vector<PointCloud::Ptr, Eigen::aligned_allocator<PointCloud::Ptr> > data;
	int red, green, blue;
	const float phaseDiff = ((2.0f*3.14159f)/3.0f);
	float completionPerc;

	// Read files
	loadData (argc, argv, data);

	// Validate user input
	if (data.empty ())
	{
		return (-1);
	}
	PCL_INFO ("Loaded %d datasets.", (int)data.size ());
	
	// Create a PCLVisualizer object
	p = new pcl::visualization::PCLVisualizer (argc, argv, "Registration Visualization");
	p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);


	PointCloud::Ptr result (new PointCloud), source, target;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
	
	red 	= (int)(sin(0) * 127.0f + 128.0f);
	green 	= (int)(sin(0 + phaseDiff) * 127.0f + 128.0f);
	blue 	= (int)(sin(0 + 2.0f*phaseDiff) * 127.0f + 128.0f);

	PointCloudColorHandlerCustom<PointT> cloud_src_h (data[0], red, green, blue);
	p->addPointCloud (data[0], cloud_src_h, "global", vp_2);

	for (size_t i = 1; i < data.size (); ++i)
	{
		std::stringstream sstream;

		source = data[i-1];
		target = data[i];

		// Add visualization data
		showCloudsLeft(source, target);

		PointCloud::Ptr temp (new PointCloud);
		PCL_INFO ("Aligning %i and %i.\n", i-1, i);
		pairAlign (source, target, temp, pairTransform, true);

		//transform current pair into the global transform
		pcl::transformPointCloud (*temp, *result, GlobalTransform);

		//update the global transform
		GlobalTransform = pairTransform * GlobalTransform;

		//pcl::transformPointCloud (*target, *temp, GlobalTransform);

		completionPerc = (((float) i+1) / ((float) data.size())) * 3.0f;
		red 	= (int)(sin(completionPerc) * 127.0f + 128.0f);
		green 	= (int)(sin(completionPerc + phaseDiff) * 127.0f + 128.0f);
		blue 	= (int)(sin(completionPerc + 2.0f*phaseDiff) * 127.0f + 128.0f);
		sstream << "capture" << i+1;
		PointCloudColorHandlerCustom<PointT> cloud_tgt_h (result, red, green, blue);
		p->addPointCloud (result, cloud_tgt_h, sstream.str(), vp_2);
		p-> spin();


		//save aligned pair, transformed into the first cloud's frame
		std::stringstream ss;
		ss << i << ".pcd";
		pcl::io::savePCDFile (ss.str (), *result, true);

	}

		showCloudsLeft(data[0], data[0]);

	cin.get();
	cin.get();
	cin.get();
}
/* ]--- */