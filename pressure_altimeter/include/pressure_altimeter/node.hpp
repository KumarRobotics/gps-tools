/*
 * node.hpp
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

#ifndef GALT_PRESSURE_ALTIMETER_NODE_HPP_
#define GALT_PRESSURE_ALTIMETER_NODE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

namespace galt {
namespace pressure_altimeter {

/**
 * @brief Converts fluid pressure message into height above sea level.
 */
class Node {
 public:
  /**
   * @brief Load parameters from ROS param and subscribe to fluid pressure.
   */
  Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  /**
   * @brief Receive pressure message and convert to height.
   * @param pressure
   */
  void pressureCallback(const sensor_msgs::FluidPressureConstPtr &pressure);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber subPressure_;
  ros::Publisher pubHeight_;
  std::string worldFrameId_;

  //  Physical constants (refer to Wikipedia for significance)
  constexpr static double kP0 = 101325;  //  Pressure at sea level (Pa)
  constexpr static double kT0 = 288.15;  //  Temperature at sea level (K)
  constexpr static double kL = 0.0065;   //  Temperature lapse rate (K/m)
  constexpr static double kG = 9.80665;  //  Gravitational acceleration (m/s^2)
  constexpr static double kM = 0.0289644;  //  Molar mass of dry air (kg / mol)
  constexpr static double kR = 8.31447;  //  Universal gas constant J/(mol * K)
};

}  //  pressure_altimeter
}  //  galt

#endif  // GALT_PRESSURE_ALTIMETER_NODE_HPP_
