/*
 * localization.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: Aaron Klein
 */


#include "localization.h"

	void Localization::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

		pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg (*input, *unfiltered_cloud);


		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (unfiltered_cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.3, 1.0);
		pass.filter (*cloud);
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (-0.5, 0.5);
		pass.filter (*cloud);
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (-0.5, 0.5);
		pass.filter (*cloud);

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr seg_inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr seg_coefficients (new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ> ());

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);

		seg.setMaxIterations (1000);
		seg.setDistanceThreshold (0.04);
		seg.setInputCloud (cloud);
		seg.segment (*seg_inliers, *seg_coefficients);

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (seg_inliers);
		extract.setNegative (true);
		extract.filter (*final);

	//	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZ> ());

	//	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//	icp.setInputCloud(_model);
	//	icp.setInputTarget(final);
	//	icp.setMaxCorrespondenceDistance (0.9);
	//	icp.setMaximumIterations (1000);
	//	icp.setTransformationEpsilon (1e-8);
	//	icp.setEuclideanFitnessEpsilon (1);
	
	//	icp.align(*aligned);

	//	std::cout << icp.getFinalTransformation() << std::endl;


	//	if(icp.hasConverged())
		{

			Eigen::VectorXf centroid;
			pcl::computeNDCentroid(*final, centroid);
	//		pcl::computeNDCentroid(*aligned, centroid);
			geometry_msgs::PoseStamped pose;
			pose.pose.position.x = centroid[0];
			pose.pose.position.y = centroid[1];
			pose.pose.position.z = centroid[2];

			pose.header.frame_id = "/camera_depth_optical_frame";
			pose.header.stamp = ros::Time::now();
			_pub_pose.publish(pose);
		
			sensor_msgs::PointCloud2 message;
			pcl::toROSMsg(*final, message);
	//		pcl::toROSMsg(*aligned, message);
			message.header.frame_id = "/camera_depth_optical_frame";
			message.header.stamp = ros::Time::now();
			_pub_cloud.publish(message);
			ROS_INFO("Pose estimated");
		}

	}

