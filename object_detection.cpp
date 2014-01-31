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


class ObjectDetection
{

public:
	ObjectDetection()
    : _it(_nh), _classifier("/home/kleinaa/devel/workspace/ObjectDetection/cascade.xml")
  {
    _rgb_image_sub = _it.subscribe("/camera/rgb/image_color", 1,&ObjectDetection::rgbImageCallback, this);
    _depth_image_sub = _it.subscribe("/camera/depth/image", 1,&ObjectDetection::depthImageCallback, this);


    _image_pub = _it.advertise("/image_converter/output_video", 1);

    cv::namedWindow("RGB Image");
    cv::namedWindow("Depth Image");
  }

  ~ObjectDetection()
  {
    cv::destroyWindow("RGB Image");
    cv::destroyWindow("Depth Image");
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

    // Update GUI Window
    cv::imshow("RGB Image", cv_ptr->image);
    cv::waitKey(3);

    _image_pub.publish(cv_ptr->toImageMsg());

  }

  void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
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

      for(unsigned i=0; i < _objects.size(); i++)
      {
      	cv::rectangle(cv_ptr->image, cv::Point(_objects[i].x, _objects[i].y),
      			cv::Point(_objects[i].x + _objects[i].width, _objects[i].y + _objects[i].height),  CV_RGB(0,255,0));
      }

       // Update GUI Window
      cv::imshow("Depth Image", cv_ptr->image);
      cv::waitKey(3);
    }

private:
  ros::NodeHandle _nh;
  image_transport::ImageTransport _it;
  image_transport::Subscriber _rgb_image_sub;
  image_transport::Subscriber _depth_image_sub;
  image_transport::Publisher _image_pub;
  cv::CascadeClassifier _classifier;
  std::vector<cv::Rect> _objects;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection");
  ObjectDetection object_detection;
  ros::spin();
  return (0);
}



