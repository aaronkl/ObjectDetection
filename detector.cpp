/*
 * detector.cpp
 *
 *  Created on: Mar 19, 2014
 *      Author: Aaron Klein
 */

#include "detection.h"s

int main(int argc, char** argv)
{
	ros::init(argc, argv, "object_detection");
	Detection det;
	ros::spin();
	return (0);
}
