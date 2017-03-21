#include <homer_mapping/slam_node.h>

SlamNode::SlamNode(ros::NodeHandle* nh) : m_HyperSlamFilter(0)
{
  init();

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
  m_SLAMMapPublisher =
      nh->advertise<nav_msgs::OccupancyGrid>("/homer_mapping/slam_map", 1);

  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.stamp = ros::Time(0);
  poseMsg.header.frame_id = "map";
  poseMsg.pose.position.x = 0.0;
  poseMsg.pose.position.y = 0.0;
  poseMsg.pose.position.z = 0.0;
  tf::Quaternion quatTF = tf::createQuaternionFromYaw(0.0);
  geometry_msgs::Quaternion quatMsg;
  tf::quaternionTFToMsg(quatTF, quatMsg);  // conversion from tf::Quaternion
                                           // to
  poseMsg.pose.orientation = quatMsg;
  m_PoseStampedPublisher.publish(poseMsg);

  ////  //broadcast transform map -> base_link
  tf::Transform transform(quatTF, tf::Vector3(0.0, 0.0, 0.0));
  m_tfBroadcaster.sendTransform(tf::StampedTransform(
      transform, poseMsg.header.stamp, "map", "base_link"));
  Pose userdef_pose(poseMsg.pose.position.x, poseMsg.pose.position.y,
                    tf::getYaw(poseMsg.pose.orientation));
  m_HyperSlamFilter->setRobotPose(userdef_pose, m_ScatterVarXY,
                                  m_ScatterVarTheta);
}

void SlamNode::init()
{
  double waitTime;
  ros::param::get("/particlefilter/wait_time", waitTime);
  m_WaitDuration = ros::Duration(waitTime);
  ros::param::get("/selflocalization/scatter_var_xy", m_ScatterVarXY);
  ros::param::get("/selflocalization/scatter_var_theta", m_ScatterVarTheta);

  m_DoMapping = true;

  int particleNum;
  ros::param::get("/particlefilter/particle_num", particleNum);
  int particleFilterNum;
  ros::param::get("/particlefilter/hyper_slamfilter/particlefilter_num",
                  particleFilterNum);
  m_HyperSlamFilter = new HyperSlamFilter(particleFilterNum, particleNum);

  m_ReferenceOdometryTime = ros::Time(0);
  m_LastMapSendTime = ros::Time(0);
  m_LastPositionSendTime = ros::Time(0);
  m_LastMappingTime = ros::Time(0);
  m_ReferenceOdometryTime = ros::Time(0);

  m_laser_queue.clear();
  m_odom_queue.clear();
}

SlamNode::~SlamNode()
{
  delete m_HyperSlamFilter;
}

void SlamNode::resetMaps()
{
  ROS_INFO("Resetting maps..");

  delete m_HyperSlamFilter;
  m_HyperSlamFilter = 0;

  init();
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.stamp = ros::Time::now();
  poseMsg.header.frame_id = "map";
  poseMsg.pose.position.x = 0.0;
  poseMsg.pose.position.y = 0.0;
  poseMsg.pose.position.z = 0.0;
  tf::Quaternion quatTF = tf::createQuaternionFromYaw(0.0);
  geometry_msgs::Quaternion quatMsg;
  tf::quaternionTFToMsg(quatTF, quatMsg);  // conversion from tf::Quaternion
                                           // to
  poseMsg.pose.orientation = quatMsg;
  m_PoseStampedPublisher.publish(poseMsg);

  m_LastLikeliestPose.set(0.0, 0.0);
  m_LastLikeliestPose.setTheta(0.0f);

  ////  //broadcast transform map -> base_link
  tf::Transform transform(quatTF, tf::Vector3(0.0, 0.0, 0.0));
  m_tfBroadcaster.sendTransform(tf::StampedTransform(
      transform, poseMsg.header.stamp, "map", "base_link"));
  Pose userdef_pose(poseMsg.pose.position.x, poseMsg.pose.position.y,
                    tf::getYaw(poseMsg.pose.orientation));
  m_HyperSlamFilter->setRobotPose(userdef_pose, m_ScatterVarXY,
                                  m_ScatterVarTheta);

  //  sendMapDataMessage();
}

void SlamNode::callbackResetHigh(const std_msgs::Empty::ConstPtr& msg)
{
  m_HyperSlamFilter->resetHigh();
}

void SlamNode::sendMapDataMessage(ros::Time mapTime)
{
  std::vector<int8_t> mapData;
  nav_msgs::MapMetaData metaData;

  OccupancyMap* occMap =
      m_HyperSlamFilter->getBestSlamFilter()->getLikeliestMap();
  occMap->getOccupancyProbabilityImage(mapData, metaData);

  nav_msgs::OccupancyGrid mapMsg;
  std_msgs::Header header;
  header.stamp = mapTime;
  header.frame_id = "map";
  mapMsg.header = header;
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
  Pose userdef_pose(msg->pose.pose.position.x, msg->pose.pose.position.y,
                    tf::getYaw(msg->pose.pose.orientation));
  m_HyperSlamFilter->setRobotPose(userdef_pose, m_ScatterVarXY,
                                  m_ScatterVarTheta);
}

void SlamNode::callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  m_laser_queue.push_back(msg);
  if (m_laser_queue.size() > 5)  // Todo param
  {
    m_laser_queue.erase(m_laser_queue.begin());
  }
}

void SlamNode::callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
  m_odom_queue.push_back(msg);
  static int last_i = 0;
  if (m_odom_queue.size() > 5)  // Todo param
  {
    last_i--;
    if (last_i < 0)
    {
      last_i = 0;
    }
    m_odom_queue.erase(m_odom_queue.begin());
  }

  static Pose last_interpolatedPose(0.0, 0.0, 0.0);
  ros::Time currentOdometryTime = msg->header.stamp;
  if ((ros::Time::now() - m_LastMappingTime > m_WaitDuration) &&
      m_odom_queue.size() > 1 && m_laser_queue.size() > 0)
  {
    int i, j;
    bool got_match = false;

    for (i = m_odom_queue.size() - 1; i > 0; i--)
    {
      // Check if we would repeat a calculation which is already done but
      // still in the queue
      if (m_odom_queue.at(i - 1)->header.stamp == m_ReferenceOdometryTime)
      {
        break;
      }

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
      last_i = i;
      m_LastLaserData = m_laser_queue.at(j);
      m_LastMappingTime = m_LastLaserData->header.stamp;

      float odoX = m_odom_queue.at(i)->pose.pose.position.x;
      float odoY = m_odom_queue.at(i)->pose.pose.position.y;
      float odoTheta = tf::getYaw(m_odom_queue.at(i)->pose.pose.orientation);
      Pose currentOdometryPose(odoX, odoY, odoTheta);
      currentOdometryTime = m_odom_queue.at(i)->header.stamp;

      odoX = m_odom_queue.at(i - 1)->pose.pose.position.x;
      odoY = m_odom_queue.at(i - 1)->pose.pose.position.y;
      odoTheta = tf::getYaw(m_odom_queue.at(i - 1)->pose.pose.orientation);
      Pose lastOdometryPose(odoX, odoY, odoTheta);
      m_ReferenceOdometryPose = lastOdometryPose;
      m_ReferenceOdometryTime = m_odom_queue.at(i - 1)->header.stamp;

      ros::Time startTime = ros::Time::now();
      // laserscan in between current odometry reading and
      // m_ReferenceOdometry
      // -> calculate pose of robot during laser scan
      ros::Duration d1 =
          m_LastLaserData->header.stamp - m_ReferenceOdometryTime;
      ros::Duration d2 = currentOdometryTime - m_ReferenceOdometryTime;

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
      ros::Duration duration = ros::Duration(0);

      last_interpolatedPose =
          m_ReferenceOdometryPose.interpolate(currentOdometryPose, timeFactor);
      m_HyperSlamFilter->filter(last_interpolatedPose, m_LastLaserData,
                                m_LastLaserData->header.stamp, duration);
      ros::Time finishTime = ros::Time::now();

      m_LastLikeliestPose =
          m_HyperSlamFilter->getBestSlamFilter()->getLikeliestPose(
              m_LastLaserData->header.stamp);

      // TODO löschen
      m_LastPositionSendTime = m_LastLaserData->header.stamp;
      // send map max. every 500 ms
      if ((m_LastLaserData->header.stamp - m_LastMapSendTime).toSec() > 0.5)
      {
        sendMapDataMessage(m_LastLaserData->header.stamp);
        m_LastMapSendTime = m_LastLaserData->header.stamp;
      }

      ROS_DEBUG_STREAM(
          "Pos. data calc time: " << (finishTime - startTime).toSec() << "s");
      ROS_DEBUG_STREAM("Map send Interval: "
                       << (finishTime - m_LastPositionSendTime).toSec() << "s");
    }
  }
  Pose currentPose = m_LastLikeliestPose;
  // currentPose.setX(currentPose.x() - last_interpolatedPose.x());
  // currentPose.setY(currentPose.y() - last_interpolatedPose.y());
  // currentPose.setTheta(currentPose.theta() -
  // last_interpolatedPose.theta());
  for (int i = (last_i - 1 < 0 ? 0 : last_i - 1); i < m_odom_queue.size(); i++)
  {
    currentPose.setX(currentPose.x() +
                     m_odom_queue.at(i)->twist.twist.linear.x);
    currentPose.setY(currentPose.y() +
                     m_odom_queue.at(i)->twist.twist.linear.y);
    currentPose.setTheta(currentPose.theta() +
                         m_odom_queue.at(i)->twist.twist.angular.z);
  }

  geometry_msgs::PoseStamped poseMsg;
  // header
  poseMsg.header.stamp = msg->header.stamp;
  poseMsg.header.frame_id = "map";

  // position and orientation
  poseMsg.pose.position.x = currentPose.x();
  poseMsg.pose.position.y = currentPose.y();
  poseMsg.pose.position.z = 0.0;
  tf::Quaternion quatTF = tf::createQuaternionFromYaw(currentPose.theta());
  geometry_msgs::Quaternion quatMsg;
  tf::quaternionTFToMsg(quatTF, quatMsg);  // conversion from tf::Quaternion
                                           // to geometry_msgs::Quaternion
  poseMsg.pose.orientation = quatMsg;
  m_PoseStampedPublisher.publish(poseMsg);
  // broadcast transform map -> base_link
  // tf::Transform transform(quatTF,tf::Vector3(currentPose.x(),
  // currentPose.y(), 0.0));
  // m_tfBroadcaster.sendTransform(tf::StampedTransform(transform,
  // msg->header.stamp, "map", "base_link"));
}

void SlamNode::callbackDoMapping(const std_msgs::Bool::ConstPtr& msg)
{
  m_DoMapping = msg->data;
  m_HyperSlamFilter->setMapping(m_DoMapping);
  ROS_INFO_STREAM("Do mapping is set to " << (m_DoMapping));
}

void SlamNode::callbackResetMap(const std_msgs::Empty::ConstPtr& msg)
{
  resetMaps();
}

void SlamNode::callbackLoadedMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  float res = msg->info.resolution;
  int height = msg->info.height;  // cell size
  int width = msg->info.width;    // cell size
  // if(height!=width) {
  // ROS_ERROR("Height != width in loaded map");
  // return;
  //}

  // convert map vector from ros format to robbie probability array
  float* map = new float[msg->data.size()];
  // generate exploredRegion
  int minX = INT_MIN;
  int minY = INT_MIN;
  int maxX = INT_MAX;
  int maxY = INT_MAX;
  for (size_t y = 0; y < msg->info.height; y++)
  {
    int yOffset = msg->info.width * y;
    for (size_t x = 0; x < msg->info.width; x++)
    {
      int i = yOffset + x;
      if (msg->data[i] == -1)
        map[i] = 0.5;
      else
        map[i] = msg->data[i] / 100.0;

      if (map[i] != 0.5)
      {
        if (minX == INT_MIN || minX > (int)x)
          minX = (int)x;
        if (minY == INT_MIN || minY > (int)y)
          minY = (int)y;
        if (maxX == INT_MAX || maxX < (int)x)
          maxX = (int)x;
        if (maxY == INT_MAX || maxY < (int)y)
          maxY = (int)y;
      }
    }
  }
  Box2D<int> exploredRegion = Box2D<int>(minX, minY, maxX, maxY);
  OccupancyMap* occMap = new OccupancyMap(map, msg->info.origin, res, width,
                                          height, exploredRegion);
  m_HyperSlamFilter->setOccupancyMap(occMap);
  m_HyperSlamFilter->setMapping(false);  // is this already done by gui message?
  ROS_INFO_STREAM("Replacing occupancy map");
}

void SlamNode::callbackMasking(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  m_HyperSlamFilter->applyMasking(msg);
}

/**
 * @brief main function
 */
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
