/*
 * object_detection.cpp
 *
 *  Created on: Jan 30, 2014
 *      Author: Aaron Klein
 */

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
	: _it(_nh), _classifier("/home/kleinaa/devel/workspace/ObjectDetection/cascade_handle_horizontal.xml")
	{
		_rgb_image_sub = _it.subscribe("/camera/rgb/image_color", 1,&Detection::rgbImageCallback, this);
		_depth_image_sub = _it.subscribe("/camera/depth_registered/image", 1,&Detection::depthImageCallback, this);
		_pub_pose = _nh.advertise<geometry_msgs::PoseStamped>("/grasp_pose", 1);
		_image_pub = _it.advertise("/image_converter/output_video", 1);
		_depth_pub = _it.advertise("/region_of_interest", 1);
		_rgb_info_sub = _nh.subscribe("/camera/rgb/camera_info", 100, &Detection::getCameraParameters, this);
		_pub_cloud = _nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
		_rgb_info_pub = _nh.advertise<sensor_msgs::CameraInfo>("/sync_camera_info",1);
		cv::namedWindow("RGB Image");
		cv::namedWindow("Depth Image");
	}

	~Detection()
	{
		cv::destroyWindow("RGB Image");
	}

	void getCameraParameters(const sensor_msgs::CameraInfo& info )
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

	void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
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

	void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		std::cout << "depth image callback" << std::endl;
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
				_info.header.stamp = time;
				cv_ptr->toImageMsg()->header.stamp = time;
				cv_ptr->toImageMsg()->header.frame_id = "/camera_depth_optical_frame";
				_info.header.frame_id = "/camera_depth_optical_frame";

				_depth_pub.publish(cv_ptr->toImageMsg());
				_rgb_info_pub.publish(_info);
			}
		}
	}

	void createPointcloudFromRegisteredDepthImage(cv::Mat& depthImage)
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

private:
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;
	image_transport::Subscriber _rgb_image_sub;
	image_transport::Subscriber _depth_image_sub;

	ros::Subscriber _rgb_info_sub;

	image_transport::Publisher _image_pub;
	image_transport::Publisher _depth_pub;
	ros::Publisher _pub_pose;
	ros::Publisher _pub_cloud;
	ros::Publisher _rgb_info_pub;
	cv::CascadeClassifier _classifier;
	std::vector<cv::Rect> _objects;
	Eigen::Matrix3f _rgbIntrinsicMatrix;
	sensor_msgs::CameraInfo _info;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Object_detection");
	Detection object_detection;
	ros::spin();
	return (0);
}



