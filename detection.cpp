/*
 * object_detection.cpp
 *
 *  Created on: Jan 30, 2014
 *      Author: Aaron Klein
 */

#include "detection.h"

void Detection::getCameraParameters(const sensor_msgs::CameraInfo& info )
{
	//TODO: set _rgbIntrinsicMatrix
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

void Detection::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
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

	std::cout << "Detected objects: " << _objects.size() << std::endl;

	for(unsigned i=0; i < _objects.size(); i++)
	{
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

	for(unsigned i=0; i < _objects.size(); i++)
	{
		cv::rectangle(cv_ptr->image, cv::Point(_objects[i].x, _objects[i].y),
				cv::Point(_objects[i].x + _objects[i].width , _objects[i].y + _objects[i].height),  CV_RGB(0,255,0));

		cv::Rect bounding_box = cv::Rect(cv::Point(_objects[i].x, _objects[i].y),
				cv::Point(_objects[i].x + _objects[i].width , _objects[i].y + _objects[i].height ));



		if(bounding_box.x >= 0 && bounding_box.y >= 0 && bounding_box.width + bounding_box.x < cv_ptr->image.cols
				&& bounding_box.height + bounding_box.y < cv_ptr->image.rows)
		{
			for(unsigned x = 0; x < cv_ptr->image.cols; x++)
			{
				for(unsigned y = 0; y < cv_ptr->image.rows; y++)
				{

					if(!(x > (_objects[i].x )&& x < (_objects[i].x + _objects[i].width )
							&& y > (_objects[i].y  + 10) &&  y < (_objects[i].y + _objects[i].height)))
					{
						cv_ptr->image.at<float>(y,x) = 0;
					}
				}
			}
			cv::imshow("Depth Image", cv_ptr->image);
			cv::waitKey(3);

			ros::Time time = ros::Time::now();
			cv_ptr->toImageMsg()->header.stamp = time;
			cv_ptr->toImageMsg()->header.frame_id = "/camera_depth_optical_frame";
			_depth_pub.publish(cv_ptr->toImageMsg());
			//				_info.header.stamp = time;

			//				_info.header.frame_id = "/camera_depth_optical_frame";


			//				_rgb_info_pub.publish(_info);
		}
	}
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

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg (*outputPointcloud, cloud_msg);
	cloud_msg.header.frame_id = "/camera_depth_optical_frame";
	cloud_msg.header.stamp = ros::Time::now();
	std::cout << "publish cloud: " <<  outputPointcloud->size() << std::endl;
	_pub_cloud.publish(cloud_msg);

}




