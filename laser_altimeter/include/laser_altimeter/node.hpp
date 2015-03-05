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
 *  This file is part of laser_altimeter.
 *
 *	Created on: 24/08/2014
 */

#ifndef GALT_LASER_ALTIMETER_NODE_HPP_
#define GALT_LASER_ALTIMETER_NODE_HPP_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>
#include <kr_math/base_types.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

namespace galt {
namespace laser_altimeter {

class Node {
 public:
  /**
   * @brief Attaches required callbacks to node handle.
   */
  Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pubHeight_;
  std::string worldFrameId_;

  double angleMin_;
  double angleMax_;

  tf2::BufferCore tfCore_;
  tf2_ros::TransformListener tfListener_;

  // kr::quatd iQl_; /// laser to IMU rotation
  // kr::vec3d iPl_; /// position of laser in imu

  message_filters::Subscriber<sensor_msgs::Imu> subImu_;
  message_filters::Subscriber<sensor_msgs::LaserScan> subScan_;

  using TimeSyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Imu, sensor_msgs::LaserScan>;
  using Synchronizer = message_filters::Synchronizer<TimeSyncPolicy>;

  std::shared_ptr<Synchronizer> sync_;

  /**
   * @brief Correct Lidar to world frame with IMU, then publish a height
   * estimate.
   */
  void syncCallback(const sensor_msgs::ImuConstPtr &imu_msg,
                    const sensor_msgs::LaserScanConstPtr &scan_msg);
};

}  //  laser_altimeter
}  //  galt

#endif  // GALT_LASER_ALTIMETER_NODE_HPP_
