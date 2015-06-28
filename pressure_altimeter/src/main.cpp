/*
 * main.cpp
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
