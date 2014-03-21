/*
 * localization.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: Aaron Klein
 */


#include "localization.h"

	void Localization::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
	{
		std::cout << "start localization" << std::endl;
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

		//			pcl::PointCloud<pcl::PointXYZ>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZ> ());
		//
		//			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		//			icp.setInputCloud(_model);
		//			icp.setInputTarget(final);
		//			icp.setMaxCorrespondenceDistance (0.9);
		//			icp.setMaximumIterations (1000);
		//			icp.setTransformationEpsilon (1e-8);
		//			icp.setEuclideanFitnessEpsilon (1);
		//			std::cout << "icp" << std::endl;
		//
		//			icp.align(*aligned);
		//
		//			std::cout << "has converged: " << icp.hasConverged() << " score: " <<
		//					icp.getFitnessScore() << std::endl;
		//			std::cout << icp.getFinalTransformation() << std::endl;
		//
		//
		//			if(icp.hasConverged())
		//			{

		Eigen::VectorXf grasp_position;
		pcl::computeNDCentroid(*final, grasp_position);
		//			pcl::computeNDCentroid(*aligned, grasp_position);
		geometry_msgs::PoseStamped grasp_pose;
		grasp_pose.pose.position.x = grasp_position[0];
		grasp_pose.pose.position.y = grasp_position[1];
		grasp_pose.pose.position.z = grasp_position[2];
		//			grasp_pose.pose.orientation.w = grasp_position[3];
		grasp_pose.header.frame_id = "/camera_depth_optical_frame";
		grasp_pose.header.stamp = ros::Time::now();

		_pub_pose.publish(grasp_pose);
		sensor_msgs::PointCloud2 message;
		pcl::toROSMsg(*final, message);
		message.header.frame_id = "/camera_depth_optical_frame";
		message.header.stamp = ros::Time::now();
		_pub_cloud.publish(message);

		//		}

	}







