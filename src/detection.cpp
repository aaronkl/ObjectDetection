/*
 * object_detection.cpp
 *
 *  Created on: Jan 30, 2014
 *      Author: Aaron Klein
 */

#include "detection.h"


void Detection::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("Start Object Detection");
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	_classifier.detectMultiScale(cv_ptr->image, _objects);

	for(unsigned i=0; i < _objects.size(); i++)
	{
		ROS_INFO("Object detected");
		cv::rectangle(cv_ptr->image, cv::Point(_objects[i].x, _objects[i].y),
				cv::Point(_objects[i].x + _objects[i].width, _objects[i].y + _objects[i].height),  CV_RGB(0,255,0));

	}
	cv::imshow("RGB Image", cv_ptr->image);
	cv::waitKey(3);
}

void Detection::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	sensor_msgs::Image ros_image;
	cv_ptr->toImageMsg(ros_image);

	unsigned step_size = ros_image.step / ros_image.width;


	for(unsigned i=0; i < _objects.size(); i++)
	{
		cv::rectangle(cv_ptr->image, cv::Point(_objects[i].x, _objects[i].y),
				cv::Point(_objects[i].x + _objects[i].width , _objects[i].y + _objects[i].height),  CV_RGB(0,255,0));

		cv::Rect bounding_box = cv::Rect(cv::Point(_objects[i].x, _objects[i].y),
				cv::Point(_objects[i].x + _objects[i].width , _objects[i].y + _objects[i].height ));

		if(bounding_box.x >= 0 && bounding_box.y >= 0 && bounding_box.width + bounding_box.x < cv_ptr->image.cols
				&& bounding_box.height + bounding_box.y < cv_ptr->image.rows)
		{
				for(unsigned y = 0; y < ros_image.height; y++)
				{
					int row_offset = y*ros_image.step;

					for(unsigned x = 0; x < ros_image.width; x++)
					{
				        	int old_index = row_offset + x*step_size;

						if(!(x > (_objects[i].x )&& x < (_objects[i].x + _objects[i].width )
							&& y > (_objects[i].y ) &&  y < (_objects[i].y + _objects[i].height)))
						{
							for(unsigned i = 0; i<step_size; i++)
								ros_image.data[old_index+i] = 0;
						}
					}
				}

			ros::Time time = ros::Time::now();
			ros_image.header.stamp = time;
			ros_image.header.frame_id = "/camera_depth_optical_frame";
			_depth_pub.publish(ros_image);

			_info.header.frame_id = "/camera_depth_optical_frame";
			_rgb_info_pub.publish(_info);

			cv_bridge::CvImage cv_image = *(cv_bridge::toCvCopy(ros_image));
			createPointcloudFromRegisteredDepthImage(cv_image.image);
		}
	}
}


void Detection::getCameraParameters(const sensor_msgs::CameraInfo& info )
{
	//     [fx  0 cx]
	// K = [ 0 fy cy]
	//     [ 0  0  1]
	_info = info;

	_rgbIntrinsicMatrix(0,0) = info.K[0];
	_rgbIntrinsicMatrix(0,1) = info.K[1];
	_rgbIntrinsicMatrix(0,2) = info.K[2];
	_rgbIntrinsicMatrix(1,0) = info.K[3];
	_rgbIntrinsicMatrix(1,1) = info.K[4];
	_rgbIntrinsicMatrix(1,2) = info.K[5];
	_rgbIntrinsicMatrix(2,0) = info.K[6];
	_rgbIntrinsicMatrix(2,1) = info.K[7];
	_rgbIntrinsicMatrix(2,2) = info.K[8];
}



void Detection::createPointcloudFromRegisteredDepthImage(cv::Mat& depthImage)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointcloud(new pcl::PointCloud<pcl::PointXYZ>);

	float rgbFocalInvertedX =  1/_rgbIntrinsicMatrix(0,0);	// 1/fx
	float rgbFocalInvertedY =  1/_rgbIntrinsicMatrix(1,1);	// 1/fy

	pcl::PointXYZ newPoint;
	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		{
			float depthValue = depthImage.at<float>(i,j);

			if (depthValue == depthValue)                // if depthValue is not NaN
			{
				// Find 3D position respect to rgb frame:
				newPoint.z =  depthValue;
				newPoint.x = (j - _rgbIntrinsicMatrix(0,2)) * depthValue * rgbFocalInvertedX;
				newPoint.y = (i - _rgbIntrinsicMatrix(1,2)) * depthValue * rgbFocalInvertedY;

				outputPointcloud->push_back(newPoint);
			}
			else
			{
				newPoint.z = std::numeric_limits<float>::quiet_NaN();
				newPoint.x = std::numeric_limits<float>::quiet_NaN();
				newPoint.y = std::numeric_limits<float>::quiet_NaN();
				outputPointcloud->push_back(newPoint);
			}
		}
	}

	outputPointcloud->header.frame_id = "/camera_depth_optical_frame";
	_points_pub.publish(*outputPointcloud);
}
