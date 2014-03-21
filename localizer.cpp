/*
 * localizer.cpp
 *
 *  Created on: Mar 19, 2014
 *      Author: Aaron Klein
 */

#include "localization.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Localization");
	Localization loc;
	ros::spin();
	return (0);
}
