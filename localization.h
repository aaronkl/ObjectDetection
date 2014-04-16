/*
 * localization.h
 *
 *  Created on: Mar 19, 2014
 *      Author: Aaron Klein
 */

#ifndef LOCALIZATION_H_
#define LOCALIZATION_H_


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
//#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/filter.h>
#include "pcl/features/normal_3d.h"
#include <pcl/registration/transforms.h>
#include "pcl/features/fpfh.h"
#include <pcl/registration/registration.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include "pcl/registration/ia_ransac.h"
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>


class Localization
{

public:
	Localization()
	:_model (new pcl::PointCloud<pcl::PointXYZ>)
	{

		_point_cloud_sub = _nh.subscribe("/object_detection/points", 1,&Localization::point_cloud_callback, this);
		_pub_pose = _nh.advertise<geometry_msgs::PoseStamped>("/object_detection/pose", 1);
		_pub_cloud = _nh.advertise<sensor_msgs::PointCloud2>("/object_detection/final", 1);

		if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/kleinaa/devel/workspace/ObjectRecognition/Handles/model.pcd", *_model) == -1)
		{
			std::cerr << "Could not load model file " << std::endl;
			exit(-1);
		}

	}

	~Localization() {}

	void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);

private:
	ros::NodeHandle _nh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _model;
	ros::Subscriber _point_cloud_sub;
	ros::Publisher _pub_pose;
	ros::Publisher _pub_cloud;
};


#endif /* LOCALIZATION_H_ */

