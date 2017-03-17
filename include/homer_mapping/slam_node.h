#ifndef SLAM_NODE_H
#define SLAM_NODE_H

#include <stdlib.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <homer_mapping/OccupancyMap/OccupancyMap.h>
#include <homer_mapping/ParticleFilter/HyperSlamFilter.h>
#include <homer_mapping/ParticleFilter/SlamFilter.h>
#include <homer_nav_libs/Math/Box2D.h>
#include <homer_nav_libs/Math/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class OccupancyMap;
class SlamFilter;
class HyperSlamFilter;

/**
 * @class SlamNode
 *
 * @author Malte Knauf, Stephan Wirth, Susanne Maur (RX), David Gossow (RX),
 *         Christian Fuchs (R12), Nicolai Wojke (R14), Susanne Thierfelder (R16)
 *
 *
 * @brief The Simultaneous localization and mapping module
 *
 * This module receives odometry and laser data and computes the
 * robot's position and a map out of this data. Then it sends a
 * geometry_msgs/PoseStamped and nav_msgs/OccupancyGrid message.
 */
class SlamNode
{
public:
  /**
   * The constructor adds the message types and prepares the module for
   * receiving them.
   */
  SlamNode(ros::NodeHandle* nh);

  /**
   * This method initializes the member variables.
   */
  virtual void init();

  /**
   * The destructor deletes the filter thread instance.
   */
  virtual ~SlamNode();

private:
  /**
   * Callback methods for all incoming messages
   */
  void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);
  void callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg);
  void callbackUserDefPose(const geometry_msgs::Pose::ConstPtr& msg);
  void callbackDoMapping(const std_msgs::Bool::ConstPtr& msg);
  void callbackResetMap(const std_msgs::Empty::ConstPtr& msg);
  void callbackLoadedMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void callbackMasking(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void callbackResetHigh(const std_msgs::Empty::ConstPtr& msg);
  void callbackInitialPose(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  /**
   * This function resets the current maps to the initial state.
   */
  void resetMaps();

  /**
   * This function processes the current odometry data in combination with the
   * last send odometry and laser informations to pass on corresponding data
   * to the filter threads.

  /**
   * This method retrieves the current map of the slam filter and sends a map
   * data message containing the map.
   */
  void sendMapDataMessage(ros::Time mapTime = ros::Time::now());

  /**
   * This variables stores the last odometry measurement as reference that is
   * used
   * to compute the pose of the robot during a specific laserscan.
   */
  Pose m_ReferenceOdometryPose;
  Pose m_lastUsedPose;

  Pose m_LastLikeliestPose;

  /**
   * This variable stores the time of the last odometry measurement as
   * reference
   * which is used to compute the pose of the robot during a specific
   * laserscan.
   */
  ros::Time m_ReferenceOdometryTime;

  /**
   * This variable stores the time the last map message was sent to be able to
   * compute the time for the next map send.
   */
  ros::Time m_LastMapSendTime;


  /**
   * This variable stores a pointer to the hyper slam filter
   */
  HyperSlamFilter* m_HyperSlamFilter;

  /**
   * Scatter variances in self localization.
   */
  double m_ScatterVarXY;
  double m_ScatterVarTheta;

  /**
   * This variabe is true, if the slam algorithm is used for mapping and
   * keeps updating the map, false otherwise.
   */
  bool m_DoMapping;

  /**
   * Vectors used to queue laser and odom messages to find a fit
   */
  std::vector<sensor_msgs::LaserScan::ConstPtr> m_laser_queue;
  std::vector<nav_msgs::Odometry::ConstPtr> m_odom_queue;

  /**
   * duration to wait between two particle filter steps
   */
  ros::Duration m_WaitDuration;

  /**
   * Broadcasts the transform map -> base_link
   */
  tf::TransformBroadcaster m_tfBroadcaster;

  /**
   * subscribers and publishers
   */
  ros::Subscriber m_LaserScanSubscriber;
  ros::Subscriber m_OdometrySubscriber;
  ros::Subscriber m_UserDefPoseSubscriber;
  ros::Subscriber m_DoMappingSubscriber;
  ros::Subscriber m_ResetMapSubscriber;
  ros::Subscriber m_LoadMapSubscriber;
  ros::Subscriber m_MaskingSubscriber;
  ros::Subscriber m_ResetHighSubscriber;
  ros::Subscriber m_InitialPoseSubscriber;

  ros::Publisher m_PoseStampedPublisher;
  ros::Publisher m_PoseArrayPublisher;
  ros::Publisher m_SLAMMapPublisher;
};

#endif
