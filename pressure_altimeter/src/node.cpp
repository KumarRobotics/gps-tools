/*
 * node.cpp
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
#include <pressure_altimeter/Height.h>  //  published message
#include <cmath>

namespace galt {
namespace pressure_altimeter {

Node::Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  subPressure_ =
      pnh_.subscribe("pressure", 1, &Node::pressureCallback, this);
  pubHeight_ = pnh_.advertise<::pressure_altimeter::Height>("height", 1);

  pnh_.param("fixed_frame", worldFrameId_, std::string("world"));
}

void Node::pressureCallback(
    const sensor_msgs::FluidPressureConstPtr &pressure) {
  ::pressure_altimeter::Height height_msg;
  height_msg.header.stamp = pressure->header.stamp;
  height_msg.header.frame_id = worldFrameId_;

  const double pressurePA = pressure->fluid_pressure * 100;  //  mb to Pa
  if (pressurePA > 0) {
    //  calculate height from barometer
    const double c = kG * kM / (kR * kL);
    const double lhs = std::exp(std::log(pressurePA / kP0) * (1 / c));
    const double h = (1 - lhs) * kT0 / kL;

    //  value of jacobian to propagate variance
    const double J = -lhs * kT0 / (kL * c * pressurePA);
    const double h_var = J * J * (pressure->variance * 100 * 100);  //  mb to Pa

    height_msg.height = h;
    height_msg.variance = h_var;
  } else {
    height_msg.height = 0;
    height_msg.variance = -1;
  }

  pubHeight_.publish(height_msg);
}

}  //  pressure_altimeter
}  //  galt
