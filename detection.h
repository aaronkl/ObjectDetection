/*
 * detection.h
 *
 *  Created on: Mar 19, 2014
 *      Author: Aaron Klein
 */

#ifndef DETECTION_H_
#define DETECTION_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
//#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


class Detection
{

public:
	Detection()
	: _it(_nh), _classifier("/home/kleinaa/devel/workspace/ObjectDetection/cascade_handle_vertical.xml")
	{
		_rgb_image_sub = _it.subscribe("/camera/rgb/image_color", 1,&Detection::rgbImageCallback, this);
		_depth_image_sub = _it.subscribe("/camera/depth_registered/image", 1,&Detection::depthImageCallback, this);
		_depth_pub = _it.advertise("/region_of_interest", 1);

		cv::namedWindow("RGB Image");
		cv::namedWindow("Depth Image");
	}

	~Detection()
	{
		cv::destroyWindow("RGB Image");
	}

	void getCameraParameters(const sensor_msgs::CameraInfo& info );

	void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);

	void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);

	void createPointcloudFromRegisteredDepthImage(cv::Mat& depthImage);

private:
	ros::NodeHandle _nh;

	image_transport::ImageTransport _it;
	image_transport::Subscriber _rgb_image_sub;
	image_transport::Subscriber _depth_image_sub;
	image_transport::Publisher _depth_pub;


	cv::CascadeClassifier _classifier;
	std::vector<cv::Rect> _objects;
};


#endif /* DETECTION_H_ */
