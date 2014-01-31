#include "isella3_controller2.h"
#include <sys/time.h>
#include <time.h>
#include <iostream>

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "isella_node");
	ROS_INFO("Node initialized");
	ros::NodeHandle n_;
	theApp.OpenDevice();
	theApp.OpenBrakes();
	while(ros::ok())
	{
	ROS_INFO("\nActual position 1: %d \nActual position 2: %d \nActual position 3: %d \nActual position 4: %d \n",(int)theApp.fGetAngleAxisIs(0,0),(int)theApp.fGetAngleAxisIs(0,1),(int)theApp.fGetAngleAxisIs(1,0),(int)theApp.fGetAngleAxisIs(1,1));
	};
	theApp.CloseBrakes();
	theApp.CloseDevice();
	ROS_INFO("End");
};
