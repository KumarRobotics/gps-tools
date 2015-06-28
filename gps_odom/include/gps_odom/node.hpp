/*
 * node.hpp
 *
 *  This file is part of gps_odom.
 *
 *	Created on: 31/07/2014
 *		  Author: gareth
 */

#ifndef GPS_ODOM_NODE_HPP_
#define GPS_ODOM_NODE_HPP_

#include <memory>

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <pressure_altimeter/Height.h>
#include "rviz_helper/marker_visualizer.hpp"
#include "rviz_helper/tf_publisher.hpp"

namespace gps_odom {

class Node {
 public:
  Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  static constexpr int kROSQueueSize = 200;

  ros::NodeHandle nh_, pnh_;
  std::string pkgPath_;
  std::string fixedFrame_;
  ros::Publisher pubOdometry_;
  ros::Publisher pubRefPoint_;

  message_filters::Subscriber<sensor_msgs::Imu> subImu_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subFix_;
  message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>
      subFixTwist_;
  message_filters::Subscriber<pressure_altimeter::Height> subHeight_;

  //  time sync policy for GPS data
  using TimeSyncGPS = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped,
      sensor_msgs::Imu, pressure_altimeter::Height>;
  using SynchronizerGPS = message_filters::Synchronizer<TimeSyncGPS>;
  std::shared_ptr<SynchronizerGPS> syncGps_;

  void gpsCallback(
      const sensor_msgs::NavSatFixConstPtr &,
      const geometry_msgs::TwistWithCovarianceStampedConstPtr &navSatTwist,
      const sensor_msgs::ImuConstPtr &,
      const pressure_altimeter::HeightConstPtr &height);

  /// Load models from disk, if required.
  bool loadModels();
  
  //  geographic lib objects
  std::shared_ptr<GeographicLib::Geoid> geoid_;
  std::shared_ptr<GeographicLib::MagneticModel> magneticModel_;
  
  GeographicLib::LocalCartesian refPoint_;
  double refPressureHeight_;
  sensor_msgs::NavSatFix refFix_;
  bool refInitialized_{false};
  
  bool refPublished_{false};
  double currentDeclination_{0}; ///< Magnetic declination from north

  //  Parameters
  double gpsCovScaleFactor_;
  bool shouldPublishTf_;
  double gpsRefLat_{0}, gpsRefLon_{0};
  bool useFixedRef_{false};

  kr::viz::TfPublisher tfPub_;
  kr::viz::TrajectoryVisualizer trajViz_;
  kr::viz::CovarianceVisualizer covViz_;
};

}  //  namespace_gps_odom

#endif  // GPS_ODOM_NODE_HPP_
