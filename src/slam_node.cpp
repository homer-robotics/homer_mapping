#include <homer_mapping/slam_node.h>

SlamNode::SlamNode(ros::NodeHandle* nh) : m_HyperSlamFilter(0)
{
  // subscribe to topics
  m_LaserScanSubscriber = nh->subscribe<sensor_msgs::LaserScan>(
      "/scan", 1, &SlamNode::callbackLaserScan, this);
  m_OdometrySubscriber = nh->subscribe<nav_msgs::Odometry>(
      "/odom", 1, &SlamNode::callbackOdometry, this);

  m_UserDefPoseSubscriber = nh->subscribe<geometry_msgs::Pose>(
      "/homer_mapping/userdef_pose", 1, &SlamNode::callbackUserDefPose, this);
  m_InitialPoseSubscriber =
      nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
          "/initialpose", 1, &SlamNode::callbackInitialPose, this);

  m_DoMappingSubscriber = nh->subscribe<std_msgs::Bool>(
      "/homer_mapping/do_mapping", 1, &SlamNode::callbackDoMapping, this);
  m_ResetMapSubscriber = nh->subscribe<std_msgs::Empty>(
      "/map_manager/reset_maps", 1, &SlamNode::callbackResetMap, this);
  m_LoadMapSubscriber = nh->subscribe<nav_msgs::OccupancyGrid>(
      "/map_manager/loaded_map", 1, &SlamNode::callbackLoadedMap, this);
  m_MaskingSubscriber = nh->subscribe<nav_msgs::OccupancyGrid>(
      "/map_manager/mask_slam", 1, &SlamNode::callbackMasking, this);
  m_ResetHighSubscriber = nh->subscribe<std_msgs::Empty>(
      "/map_manager/reset_high", 1, &SlamNode::callbackResetHigh, this);

  // advertise topics
  m_PoseStampedPublisher =
      nh->advertise<geometry_msgs::PoseStamped>("/pose", 2);
  m_PoseArrayPublisher =
      nh->advertise<geometry_msgs::PoseArray>("/pose_array", 2);
  m_SLAMMapPublisher = nh->advertise<nav_msgs::OccupancyGrid>(
      "/homer_mapping/slam_map", 1, true);


  bool base_fp_param;
  if (nh->hasParam("/home_mapping/map_childof_base_footprint"))
    if (ros::param::get("/homer_mapping/map_childof_base_footprint", base_fp_param))
    {
       m_UseBaseFootprint = base_fp_param;
    }
 else
    m_UseBaseFootprint = false;
  init();
}

void SlamNode::init()
{
  ros::param::get("/selflocalization/scatter_var_xy", m_ScatterVarXY);
  ros::param::get("/selflocalization/scatter_var_theta", m_ScatterVarTheta);

  m_DoMapping = true;

  int particleNum;
  int particleFilterNum;
  ros::param::get("/particlefilter/particle_num", particleNum);
  ros::param::get("/particlefilter/hyper_slamfilter/particlefilter_num",
                  particleFilterNum);
  m_HyperSlamFilter = new HyperSlamFilter(particleFilterNum, particleNum);

  m_LastMapSendTime = ros::Time(0);

  m_laser_queue.clear();
  m_odom_queue.clear();

  sendTfAndPose(Pose(0, 0, 0), ros::Time::now());
  m_HyperSlamFilter->setRobotPose(Pose(0, 0, 0), m_ScatterVarXY,
                                  m_ScatterVarTheta);
}

SlamNode::~SlamNode()
{
  delete m_HyperSlamFilter;
}

void SlamNode::resetMaps()
{
  ROS_INFO("Resetting maps..");

  if (m_HyperSlamFilter)
  {
    delete m_HyperSlamFilter;
  }
  m_HyperSlamFilter = 0;

  init();
}

void SlamNode::callbackResetHigh(const std_msgs::Empty::ConstPtr& msg)
{
  m_HyperSlamFilter->resetHigh();
}

void SlamNode::sendMapDataMessage(ros::Time mapTime)
{
  std::vector<int8_t> mapData;
  nav_msgs::MapMetaData metaData;

  m_HyperSlamFilter->getBestSlamFilter()
      ->getLikeliestMap()
      ->getOccupancyProbabilityImage(mapData, metaData);

  nav_msgs::OccupancyGrid mapMsg;
  mapMsg.header.stamp = mapTime;
  mapMsg.header.frame_id = "map";
  mapMsg.info = metaData;
  mapMsg.data = mapData;

  m_SLAMMapPublisher.publish(mapMsg);
}

void SlamNode::callbackUserDefPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  Pose userdef_pose(msg->position.x, msg->position.y,
                    tf::getYaw(msg->orientation));
  m_HyperSlamFilter->setRobotPose(userdef_pose, m_ScatterVarXY,
                                  m_ScatterVarTheta);
}

void SlamNode::callbackInitialPose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	tf::StampedTransform transform;
	try {
        if (m_tfListener.waitForTransform("/map", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0) )) {
		    m_tfListener.lookupTransform("/map", msg->header.frame_id, msg->header.stamp, transform);
        } else {
		    m_tfListener.lookupTransform("/map", msg->header.frame_id, ros::Time(0), transform);
        }
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}
    tf::Vector3 targetPos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    targetPos = transform * targetPos;
    double targetYaw = tf::getYaw(transform * tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    Pose userdef_pose(targetPos.getX(), targetPos.getY(), targetYaw);
    m_HyperSlamFilter->setRobotPose(userdef_pose, m_ScatterVarXY, m_ScatterVarTheta);
}

void SlamNode::callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  m_laser_queue.push_back(msg);
  if (m_laser_queue.size() > 5)
  {
    m_laser_queue.erase(m_laser_queue.begin());
  }
}

void SlamNode::sendTfAndPose(Pose pose, ros::Time stamp)
{
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.stamp = stamp;
  poseMsg.header.frame_id = "map";
  poseMsg.pose.position.x = pose.x();
  poseMsg.pose.position.y = pose.y();
  poseMsg.pose.position.z = 0.0;
  tf::Quaternion quatTF = tf::createQuaternionFromYaw(pose.theta());
  geometry_msgs::Quaternion quatMsg;
  tf::quaternionTFToMsg(quatTF, quatMsg);

  poseMsg.pose.orientation = quatMsg;
  m_PoseStampedPublisher.publish(poseMsg);

  // Publish Transform differetn for tiago and lisa
  tf::Transform transform(quatTF, tf::Vector3(pose.x(), pose.y(), 0.0));
 // ROS_INFO_STREAM("TF before inverse");
//  ROS_INFO_STREAM(transform);
 // ROS_INFO_STREAM("After");
  
  if (m_UseBaseFootprint)
  {
       tf::Transform transform2 = transform.inverse();
       // ROS_INFO_STREAM(transform2);
       m_tfBroadcaster.sendTransform(
       tf::StampedTransform(transform2, stamp, "base_footprint", "map"));
  }
  else 
  {
       m_tfBroadcaster.sendTransform(
       tf::StampedTransform(transform, stamp, "map", "base_link")); 
  }

}

void SlamNode::sendPoseArray(std::vector<Pose> poses)
{
  if (m_PoseArrayPublisher.getNumSubscribers() > 0)
  {
    // Pose Array publishing
    geometry_msgs::PoseArray poseArray = geometry_msgs::PoseArray();
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "/map";

    for (int i = 0; i < poses.size(); i++)
    {
      geometry_msgs::Pose tmpPose = geometry_msgs::Pose();
      tmpPose.position.x = poses.at(i).x();
      tmpPose.position.y = poses.at(i).y();
      tf::Quaternion quatTF = tf::createQuaternionFromYaw(poses.at(i).theta());
      geometry_msgs::Quaternion quatMsg;
      tf::quaternionTFToMsg(quatTF, quatMsg);
      tmpPose.orientation = quatMsg;
      poseArray.poses.push_back(tmpPose);
    }
    m_PoseArrayPublisher.publish(poseArray);
  }
}

void SlamNode::callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
  m_odom_queue.push_back(msg);
  if (m_odom_queue.size() > 20)
  {
    m_odom_queue.erase(m_odom_queue.begin());
  }

  if (m_odom_queue.size() > 1 && m_laser_queue.size() > 0)
  {
    int i, j;
    bool got_match = false;

    for (i = m_odom_queue.size() - 1; i > 0; i--)
    {
      for (j = m_laser_queue.size() - 1; j > -1; j--)
      {
        //	find a laserscan in between two odometry readings (or at
        // the same time)
        if ((m_laser_queue.at(j)->header.stamp >=
             m_odom_queue.at(i - 1)->header.stamp) &&
            (m_odom_queue.at(i)->header.stamp >=
             m_laser_queue.at(j)->header.stamp))
        {
          got_match = true;
          break;
        }
      }
      if (got_match)
      {
        break;
      }
    }

    if (got_match)
    {
      sensor_msgs::LaserScan::ConstPtr laserData = m_laser_queue.at(j);

      Pose last_interpolatedPose = getInterpolatedPose(
          m_odom_queue.at(i - 1), m_odom_queue.at(i), laserData->header.stamp);

      Transformation2D trans = last_interpolatedPose - m_lastUsedPose;
      m_lastUsedPose = last_interpolatedPose;

      // Rotate transformation to pose theta
      float x = trans.x() * cos(-last_interpolatedPose.theta()) -
                trans.y() * sin(-last_interpolatedPose.theta());
      float y = trans.y() * cos(-last_interpolatedPose.theta()) +
                trans.x() * sin(-last_interpolatedPose.theta());

      Transformation2D rotTrans(x, y, trans.theta());

      // SLAM STEP
      m_HyperSlamFilter->filter(rotTrans, laserData);

      Pose pose = m_HyperSlamFilter->getBestSlamFilter()->getLikeliestPose();

      sendTfAndPose(pose, laserData->header.stamp);

      // send map max. every 500 ms
      if ((laserData->header.stamp - m_LastMapSendTime).toSec() > 0.5 &&
          m_DoMapping)
      {
        sendMapDataMessage(laserData->header.stamp);
        m_LastMapSendTime = laserData->header.stamp;
      }
      sendPoseArray(m_HyperSlamFilter->getBestSlamFilter()->getParticlePoses());

      // Remove already used data from queues
      m_odom_queue.erase(m_odom_queue.begin(), m_odom_queue.begin() + (i - 1));
      m_laser_queue.erase(m_laser_queue.begin(), m_laser_queue.begin() + j);
    }
  }
}

Pose SlamNode::getInterpolatedPose(nav_msgs::Odometry::ConstPtr pose1,
                                   nav_msgs::Odometry::ConstPtr pose2,
                                   ros::Time laserTime)
{
  ros::Time currentOdometryTime = pose2->header.stamp;
  ros::Time lastOdometryTime = pose1->header.stamp;

  Pose lastOdometryPose(pose1->pose.pose.position.x,
                        pose1->pose.pose.position.y,
                        tf::getYaw(pose1->pose.pose.orientation));

  Pose currentOdometryPose(pose2->pose.pose.position.x,
                           pose2->pose.pose.position.y,
                           tf::getYaw(pose2->pose.pose.orientation));

  ros::Duration d1 = laserTime - lastOdometryTime;
  ros::Duration d2 = currentOdometryTime - lastOdometryTime;

  float timeFactor;

  if (d1.toSec() == 0.0)
  {
    timeFactor = 0.0f;
  }
  else if (d2.toSec() == 0.0)
  {
    timeFactor = 1.0f;
  }
  else
  {
    timeFactor = d1.toSec() / d2.toSec();
  }

  return lastOdometryPose.interpolate(currentOdometryPose, timeFactor);
}

void SlamNode::callbackDoMapping(const std_msgs::Bool::ConstPtr& msg)
{
  m_DoMapping = msg->data;
  m_HyperSlamFilter->setMapping(m_DoMapping);
  ROS_INFO_STREAM("Do mapping is set to " << (m_DoMapping));
}

void SlamNode::callbackResetMap(const std_msgs::Empty::ConstPtr& msg)
{
  (void)msg;
  resetMaps();
}

void SlamNode::callbackLoadedMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  OccupancyMap* occMap = new OccupancyMap(msg);
  m_HyperSlamFilter->setOccupancyMap(occMap);
  m_HyperSlamFilter->setMapping(false);
  m_DoMapping = false;
  ROS_INFO_STREAM("Replacing occupancy map");
  sendMapDataMessage(msg->info.map_load_time);
}

void SlamNode::callbackMasking(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  m_HyperSlamFilter->applyMasking(msg);
  sendMapDataMessage(ros::Time::now());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "homer_mapping");
  ros::NodeHandle nh;

  SlamNode slamNode(&nh);

  ros::Rate loop_rate(160);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
