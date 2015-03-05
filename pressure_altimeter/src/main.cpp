/*
 * main.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of pressure_altimeter.
 *
 *	Created on: 24/08/2014
 */

#include <pressure_altimeter/node.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pressure_altimeter");
  ros::NodeHandle nh, pnh("~");
  try {
    galt::pressure_altimeter::Node node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
