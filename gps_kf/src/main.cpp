/*
 * main.cpp
 *
 *  This file is part of gps_odom.
 *
 *	Created on: 03/08/2014
 *		  Author: gareth
 */

#include <ros/ros.h>
#include <gps_kf/node.hpp>

int main(int argc, char ** argv) {
  ros::init(argc,argv,"gps_kf");
  gps_kf::Node node;
  node.initialize();
  ros::spin();
}
