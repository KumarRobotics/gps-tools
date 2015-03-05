/*
 * main.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of gps_odom.
 *
 *	Created on: 31/07/2014
 *		  Author: gareth
 */

#include <ros/ros.h>
#include <gps_odom/node.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_odom");
  ros::NodeHandle nh, pnh("~");
  try {
    gps_odom::Node node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
