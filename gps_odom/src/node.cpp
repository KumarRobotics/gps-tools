/*
 * node.cpp
 *
 *  Copyright (c) 2013 Nouka Technologies. All rights reserved.
 *
 *  This file is part of gps_odom.
 *
 *	Created on: 31/07/2014
 *		  Author: gareth
 */

#include <gps_odom/node.hpp>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <kr_math/SO3.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace gps_odom {

Node::Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh),
      pkgPath_(ros::package::getPath("gps_odom")),
      trajViz_(nh_),
      covViz_(nh_) {
  if (pkgPath_.empty()) {
    ROS_WARN("Failed to find path for package");
  }
  //  ID to place on outgoing odometry
  pnh_.param<std::string>("fixed_frame", fixedFrame_, "world");

  //  scale factor for GPS covariance
  pnh_.param<double>("gps_covariance_scale_factor", gpsCovScaleFactor_, 1.0);
  //  do we wait for laser altimeter to initialize height?
  pnh_.param<bool>("laser_init", shouldUseLaserInit_, false);
  pnh_.param<bool>("publish_tf", shouldPublishTf_, false);

  //  IMU and GPS are synchronized separately
  subFix_.subscribe(nh_, "fix", kROSQueueSize);
  subFixTwist_.subscribe(nh_, "fix_velocity", kROSQueueSize);

  subImu_.subscribe(pnh_, "imu", kROSQueueSize);
  subHeight_.subscribe(pnh_, "pressure_height", kROSQueueSize);
  subLaserHeight_ =
      pnh_.subscribe("laser_height", 1, &Node::laserAltCallback, this);

  syncGps_ = std::make_shared<SynchronizerGPS>(
      TimeSyncGPS(kROSQueueSize), subFix_, subFixTwist_, subImu_, subHeight_);
  syncGps_->registerCallback(
      boost::bind(&Node::gpsCallback, this, _1, _2, _3, _4));

  //  advertise odometry as output
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
  //  reference point on a latched topic
  pubRefPoint_ = nh_.advertise<sensor_msgs::NavSatFix>("reference", 1, true);

  ROS_INFO("Using a GPS covariance scale factor of %f", gpsCovScaleFactor_);
  refGpsSet_ = false;
  refLaserSet_ = false;
  refLaserHeight_ = 0;
  currentDeclination_ = 0.0;
  trajViz_.SetColor(kr::viz::colors::RED);
  trajViz_.SetAlpha(1);
  covViz_.SetColor(kr::viz::colors::RED);
  covViz_.SetAlpha(0.5);
}

void Node::gpsCallback(
    const sensor_msgs::NavSatFixConstPtr &navSatFix,
    const geometry_msgs::TwistWithCovarianceStampedConstPtr &navSatTwist,
    const sensor_msgs::ImuConstPtr &imu,
    const pressure_altimeter::HeightConstPtr &height) {

  if (shouldUseLaserInit_ && !refLaserSet_) {
    //  we are waiting for laser initialization height
    ROS_INFO_THROTTLE(5.0,
                      "gps_odom has not yet received laser initialization");
    return;
  }

  const double lat = navSatFix->latitude;
  const double lon = navSatFix->longitude;
  const double hWGS84 = navSatFix->altitude;
  const double tYears =
      navSatFix->header.stamp.toSec() / (365 * 24 * 3600.0) + 1970;

  //  load geoid if required
  if (!geoid_) {
    try {
      geoid_ = std::make_shared<GeographicLib::Geoid>("egm84-15",
                                                      pkgPath_ + "/geoids");
    }
    catch (GeographicLib::GeographicErr &e) {
      ROS_ERROR("Failed to load geoid. Reason: %s", e.what());
      return;
    }
  }
  //  load magnetic model, if required
  if (!magneticModel_) {
    try {
      magneticModel_ = std::make_shared<GeographicLib::MagneticModel>(
          "wmm2010", pkgPath_ + "/magnetic_models");
    }
    catch (GeographicLib::GeographicErr &e) {
      ROS_ERROR("Failed to load model. Reason: %s", e.what());
      return;
    }
  }

  //  convert to height above sea level
  const double hMSL = geoid_->ConvertHeight(
      lat, lon, hWGS84, GeographicLib::Geoid::ELLIPSOIDTOGEOID);

  //  generate linear coordinates
  if (!refGpsSet_) {
    refPoint_ = GeographicLib::LocalCartesian(lat, lon, 0);
    refPressureHeight_ = height->height;
    refGpsSet_ = true;

    //  publish only once
    sensor_msgs::NavSatFix refFix = *navSatFix;
    refFix.header.seq = 0;
    pubRefPoint_.publish(refFix);
  }

  double locX, locY, locZ;
  refPoint_.Forward(lat, lon, hMSL, locX, locY, locZ);

  //  determine magnetic declination
  double bEast, bNorth, bUp;
  magneticModel_->operator()(tYears, lat, lon, hWGS84, bEast, bNorth, bUp);
  currentDeclination_ = -std::atan2(bEast, bNorth);
  const double degDec = currentDeclination_ * 180 / M_PI;
  ROS_INFO_ONCE_NAMED("gps_odom_mag_dec",
                      "Magnetic declination set to %f degrees", degDec);

  //  calculate corrected yaw angle
  //  matrix composition is of the form wRb = Rz * Ry * Rx
  Eigen::AngleAxisd aa;
  aa.angle() = currentDeclination_;
  aa.axis() = Eigen::Vector3d(0.0, 0.0, 1.0);

  Eigen::Quaterniond wQb =
      Eigen::Quaterniond(imu->orientation.w, imu->orientation.x,
                         imu->orientation.y, imu->orientation.z);
  wQb = aa * wQb;

  nav_msgs::Odometry odometry;
  odometry.header.stamp = navSatFix->header.stamp;
  odometry.header.frame_id = fixedFrame_;
  odometry.child_frame_id = fixedFrame_;
  odometry.pose.pose.orientation.w = wQb.w();
  odometry.pose.pose.orientation.x = wQb.x();
  odometry.pose.pose.orientation.y = wQb.y();
  odometry.pose.pose.orientation.z = wQb.z();
  odometry.pose.pose.position.x = locX;
  odometry.pose.pose.position.y = locY;
  odometry.pose.pose.position.z =
      (height->height - refPressureHeight_ + refLaserHeight_);

  //  generate covariance (6x6 with order: x,y,z,rot_x,rot_y,rot_z)
  Eigen::Matrix<double, 6, 6> poseCovariance;
  poseCovariance.setZero();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      poseCovariance(i, j) =
          navSatFix->position_covariance[i * 3 + j] * gpsCovScaleFactor_;
    }
  }
  //  replace covariance w/ number from altimeter
  poseCovariance(2, 2) = height->variance;

  //  orientation: copy from IMU
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      poseCovariance(3 + i, 3 + j) = imu->orientation_covariance[i * 3 + j];
    }
  }

  //  linear velocity from GPS, no angular velocity
  odometry.twist.twist.linear = navSatTwist->twist.twist.linear;
  odometry.twist.twist.angular.x = 0;
  odometry.twist.twist.angular.y = 0;
  odometry.twist.twist.angular.z = 0;

  //  velocity covariance [x,y,z,rot_x,rot_y,rot_z]
  //  linear from GPS, no angular
  Eigen::Matrix<double, 6, 6> velCovariance;
  velCovariance.setZero();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      velCovariance(i, j) =
          navSatTwist->twist.covariance[(i * 6) + j] * gpsCovScaleFactor_;
      velCovariance(i + 3, j + 3) = -1;  //  unsupported
    }
  }

  //  copy covariance to output
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      odometry.pose.covariance[i * 6 + j] = poseCovariance(i, j);
      odometry.twist.covariance[i * 6 + j] = velCovariance(i, j);
    }
  }
  pubOdometry_.publish(odometry);

  //  publish tf stuff and trajectory visualizer
  if (shouldPublishTf_) {
    tfPub_.set_child_frame_id(imu->header.frame_id);
    tfPub_.PublishTransform(odometry.pose.pose, odometry.header);
  }
  trajViz_.PublishTrajectory(odometry.pose.pose.position, odometry.header);
  covViz_.PublishCovariance(odometry);
}

void Node::laserAltCallback(
    const laser_altimeter::HeightConstPtr &laserHeight) {
  if (shouldUseLaserInit_) {
    refLaserHeight_ = laserHeight->max;
    refLaserSet_ = true;
    ROS_INFO_ONCE_NAMED("gps_odom_ref_height",
                        "Initialized gps_odom w/ laser height: %f",
                        refLaserHeight_);
  }
}

}  //  namespace gps_odom
