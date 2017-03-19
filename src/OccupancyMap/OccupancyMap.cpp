#include <homer_mapping/OccupancyMap/OccupancyMap.h>
#include <homer_nav_libs/tools.h>

using namespace std;

OccupancyMap::OccupancyMap()
{
  float mapSize, resolution;
  ros::param::get("/homer_mapping/size", mapSize);
  ros::param::get("/homer_mapping/resolution", resolution);

  // add one safety pixel
  m_metaData.resolution = resolution;
  m_metaData.width = mapSize / m_metaData.resolution + 1;
  m_metaData.height = mapSize / m_metaData.resolution + 1;
  m_ByteSize = m_metaData.width * m_metaData.height;

  m_metaData.origin.position.x =
      -(m_metaData.width * m_metaData.resolution / 2.0);
  m_metaData.origin.position.y =
      -(m_metaData.height * m_metaData.resolution / 2.0);
  m_metaData.origin.orientation.w = 1.0;
  m_metaData.origin.orientation.x = 0.0;
  m_metaData.origin.orientation.y = 0.0;
  m_metaData.origin.orientation.z = 0.0;
  initMembers();
}

OccupancyMap::OccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  m_metaData = msg->info;
  m_ByteSize = m_metaData.width * m_metaData.height;
  initMembers();

  for (unsigned i = 0; i < msg->data.size(); i++)
  {
    if (msg->data[i] != -1)
    {
      m_MapPoints[i].OccupancyProbability = msg->data[i] / 100.0;
      m_MapPoints[i].MeasurementCount = LOADED_MEASURECOUNT;
      m_MapPoints[i].OccupancyCount =
          m_MapPoints[i].OccupancyProbability * LOADED_MEASURECOUNT;
    }
  }
}

OccupancyMap::OccupancyMap(const OccupancyMap& occupancyMap)
{
  *this = occupancyMap;
}

OccupancyMap::~OccupancyMap()
{
}

void OccupancyMap::initMembers()
{
  ros::param::get("/homer_mapping/backside_checking", m_BacksideChecking);
  ros::param::get("/homer_mapping/obstacle_borders", m_ObstacleBorders);
  ros::param::get("/homer_mapping/measure_sampling_step",
                  m_MeasureSamplingStep);
  ros::param::get("/homer_mapping/laser_scanner/free_reading_distance",
                  m_FreeReadingDistance);

  m_MapPoints.resize(m_ByteSize);

  m_ExploredRegion =
      Box2D<int>(m_metaData.width / 2.1, m_metaData.height / 2.1,
                 m_metaData.width / 1.9, m_metaData.height / 1.9);
  maximizeChangedRegion();
}

OccupancyMap& OccupancyMap::operator=(const OccupancyMap& occupancyMap)
{
  m_metaData = occupancyMap.m_metaData;
  m_ExploredRegion = occupancyMap.m_ExploredRegion;
  m_ByteSize = occupancyMap.m_ByteSize;

  m_MapPoints = occupancyMap.m_MapPoints;

  return *this;
}

void OccupancyMap::changeMapSize(int x_add_left, int y_add_up, int x_add_right,
                                 int y_add_down)
{
  int new_width = m_metaData.width + x_add_left + x_add_right;
  int new_height = m_metaData.height + y_add_up + y_add_down;

  m_ByteSize = new_width * new_height;

  std::vector<PixelValue> tmpMap;
  tmpMap.resize(m_ByteSize);

  for (int y = 0; y < m_metaData.height; y++)
  {
    for (int x = 0; x < m_metaData.width; x++)
    {
      int i = y * m_metaData.width + x;
      int in = (y + y_add_up) * new_width + (x + x_add_left);
      tmpMap[in] = m_MapPoints[i];
    }
  }

  m_ExploredRegion.setMinX(m_ExploredRegion.minX() + x_add_left);
  m_ExploredRegion.setMaxX(m_ExploredRegion.maxX() + x_add_left);
  m_ExploredRegion.setMinY(m_ExploredRegion.minY() + y_add_up);
  m_ExploredRegion.setMaxY(m_ExploredRegion.maxY() + y_add_up);

  m_ChangedRegion.setMinX(m_ChangedRegion.minX() + x_add_left);
  m_ChangedRegion.setMaxX(m_ChangedRegion.maxX() + x_add_left);
  m_ChangedRegion.setMinY(m_ChangedRegion.minY() + y_add_up);
  m_ChangedRegion.setMaxY(m_ChangedRegion.maxY() + y_add_up);

  m_metaData.width = new_width;
  m_metaData.height = new_height;
  m_metaData.origin.position.x -= (x_add_left)*m_metaData.resolution;
  m_metaData.origin.position.y -= (y_add_up)*m_metaData.resolution;

  m_MapPoints = tmpMap;
}

int OccupancyMap::width() const
{
  return m_metaData.width;
}

int OccupancyMap::height() const
{
  return m_metaData.height;
}

float OccupancyMap::getOccupancyProbability(Eigen::Vector2i p)
{
  if (p.y() >= m_metaData.height || p.x() >= m_metaData.width)
  {
    return UNKNOWN_LIKELIHOOD;
  }
  unsigned offset = m_metaData.width * p.y() + p.x();
  return m_MapPoints[offset].OccupancyProbability;
}

void OccupancyMap::resetHighSensitive()
{
  ROS_INFO_STREAM("High sensitive Areas reseted");
  m_reset_high = true;
}

void OccupancyMap::computeOccupancyProbabilities()
{
  for (int y = m_ChangedRegion.minY(); y <= m_ChangedRegion.maxY(); y++)
  {
    int yOffset = m_metaData.width * y;
    for (int x = m_ChangedRegion.minX(); x <= m_ChangedRegion.maxX(); x++)
    {
      int i = x + yOffset;
      if (m_MapPoints[i].MeasurementCount > 0)
      {
        m_MapPoints[i].OccupancyProbability =
            m_MapPoints[i].OccupancyCount /
            static_cast<float>(m_MapPoints[i].MeasurementCount);
        if (m_MapPoints[i].HighSensitive == 1)
        {
          if (m_reset_high == true)
          {
            m_MapPoints[i].OccupancyCount = 0;
            m_MapPoints[i].OccupancyProbability = 0;
          }
          if (m_MapPoints[i].MeasurementCount > 20)
          {
            m_MapPoints[i].MeasurementCount = 10;
            m_MapPoints[i].OccupancyCount =
                10 * m_MapPoints[i].OccupancyProbability;
          }
          if (m_MapPoints[i].OccupancyProbability > 0.3)
          {
            m_MapPoints[i].OccupancyProbability = 1;
          }
        }
      }
      else
      {
        m_MapPoints[i].OccupancyProbability = UNKNOWN_LIKELIHOOD;
      }
    }
  }
  if (m_reset_high == true)
  {
    m_reset_high = false;
  }
}

void OccupancyMap::insertLaserData(sensor_msgs::LaserScan::ConstPtr laserData,
                                   tf::Transform transform)
{
  m_latestMapTransform = transform;
  try
  {
    m_tfListener.waitForTransform("/base_link", laserData->header.frame_id,
                                  ros::Time(0), ros::Duration(0.2));
    m_tfListener.lookupTransform("/base_link", laserData->header.frame_id,
                                 ros::Time(0), m_laserTransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_STREAM(ex.what());
    ROS_ERROR_STREAM("need transformation from base_link to laser!");
    return;
  }
  markRobotPositionFree();

  std::vector<RangeMeasurement> ranges;

  bool errorFound = false;
  int lastValidIndex = -1;
  float lastValidRange = m_FreeReadingDistance;

  RangeMeasurement rangeMeasurement;
  rangeMeasurement.sensorPos.x = m_laserTransform.getOrigin().getX();
  rangeMeasurement.sensorPos.y = m_laserTransform.getOrigin().getY();

  for (unsigned int i = 0; i < laserData->ranges.size(); i++)
  {
    if ((laserData->ranges[i] >= laserData->range_min) &&
        (laserData->ranges[i] <= laserData->range_max))
    {
      // if we're at the end of an errorneous segment, interpolate
      // between last valid point and current point
      if (errorFound)
      {
        // smaller of the two ranges belonging to end points
        float range = std::min(lastValidRange, laserData->ranges[i]);
        range -= m_metaData.resolution * 2;
        if (range < m_FreeReadingDistance)
        {
          range = m_FreeReadingDistance;
        }
        else if (range > laserData->range_max)
        {
          range = laserData->range_max;
        }
        // choose smaller range
        for (unsigned j = lastValidIndex + 1; j < i; j++)
        {
          float alpha = laserData->angle_min + j * laserData->angle_increment;
          tf::Vector3 pin;
          tf::Vector3 pout;
          pin.setX(cos(alpha) * range);
          pin.setY(sin(alpha) * range);
          pin.setZ(0);
          pout = m_laserTransform * pin;
          rangeMeasurement.endPos.x = pout.x();
          rangeMeasurement.endPos.y = pout.y();
          rangeMeasurement.range = range;
          rangeMeasurement.free = true;
          ranges.push_back(rangeMeasurement);
        }
      }
      float alpha = laserData->angle_min + i * laserData->angle_increment;
      tf::Vector3 pin;
      tf::Vector3 pout;
      pin.setX(cos(alpha) * laserData->ranges[i]);
      pin.setY(sin(alpha) * laserData->ranges[i]);
      pin.setZ(0);
      pout = m_laserTransform * pin;
      rangeMeasurement.endPos.x = pout.x();
      rangeMeasurement.endPos.y = pout.y();
      rangeMeasurement.range = laserData->ranges[i];
      rangeMeasurement.free = false;
      ranges.push_back(rangeMeasurement);
      errorFound = false;
      lastValidIndex = i;
      lastValidRange = laserData->ranges[i];
    }
    else
    {
      errorFound = true;
    }
  }
  insertRanges(ranges, laserData->header.stamp);
}

void OccupancyMap::insertRanges(vector<RangeMeasurement> ranges,
                                ros::Time stamp)
{
  if (ranges.size() < 1)
  {
    return;
  }
  clearChanges();

  Eigen::Vector2i lastEndPixel;
  int need_x_left = 0;
  int need_x_right = 0;
  int need_y_up = 0;
  int need_y_down = 0;

  std::vector<Eigen::Vector2i> map_pixel;

  for (int i = 0; i < ranges.size(); i++)
  {
    geometry_msgs::Point endPosWorld =
        map_tools::transformPoint(ranges[i].endPos, m_latestMapTransform);
    map_pixel.push_back(map_tools::toMapCoords(endPosWorld, m_metaData.origin,
                                               m_metaData.resolution));
  }
  geometry_msgs::Point sensorPosWorld =
      map_tools::transformPoint(ranges[0].sensorPos, m_latestMapTransform);
  Eigen::Vector2i sensorPixel = map_tools::toMapCoords(
      sensorPosWorld, m_metaData.origin, m_metaData.resolution);
  m_ChangedRegion.enclose(sensorPixel.x(), sensorPixel.y());

  // paint safety borders
  if (m_ObstacleBorders)
  {
    for (unsigned i = 0; i < ranges.size(); i++)
    {
      if (i < 2 || i > ranges.size() - 2)
      {
        continue;
      }
      Eigen::Vector2i endPixel = map_pixel[i];

      for (int y = endPixel.y() - 2; y <= endPixel.y() + 2; y++)
      {
        if (y > m_metaData.height || y < 0)
          continue;
        for (int x = endPixel.x() - 2; x <= endPixel.x() + 2; x++)
        {
          if (x > m_metaData.width || x < 0)
            continue;
          unsigned offset = x + m_metaData.width * y;
          if (offset < unsigned(m_ByteSize))
          {
            m_MapPoints[offset].CurrentChange = ::CONTRAST;
          }
        }
      }
    }
  }
  ////paint safety ranges
  // for ( unsigned i=0; i<ranges.size(); i++ )
  //{
  // Eigen::Vector2i startPixel = map_pixel[i];
  // geometry_msgs::Point endPos;
  // endPos.x = ranges[i].endPos.x * 4;
  // endPos.y = ranges[i].endPos.y * 4;

  // geometry_msgs::Point endPosWorld = map_tools::transformPoint(endPos,
  // m_latestMapTransform);
  // Eigen::Vector2i endPixel = map_tools::toMapCoords(endPosWorld,
  // m_metaData.origin, m_metaData.resolution);

  // if(endPixel.x() < 0) endPixel.x() = 0;
  // if(endPixel.y() < 0) endPixel.y() = 0;
  // if(endPixel.x() >= m_metaData.width) endPixel.x() = m_metaData.width - 1;
  // if(endPixel.y() >= m_metaData.height) endPixel.y() = m_metaData.height -
  // 1;

  // drawLine (  startPixel, endPixel, SAFETY_BORDER );
  //}

  // paint end pixels
  for (unsigned i = 0; i < ranges.size(); i++)
  {
    Eigen::Vector2i endPixel = map_pixel[i];

    if (endPixel != lastEndPixel)
    {
      bool outside = false;
      if (endPixel.x() >= m_metaData.width)
      {
        need_x_right =
            std::max((int)(endPixel.x() - m_metaData.width + 10), need_x_right);
        outside = true;
      }
      if (endPixel.x() < 0)
      {
        need_x_left = std::max((int)(-endPixel.x()) + 10, need_x_left);
        outside = true;
      }
      if (endPixel.y() >= m_metaData.height)
      {
        need_y_down =
            std::max((int)(endPixel.y() - m_metaData.height) + 10, need_y_down);
        outside = true;
      }
      if (endPixel.y() < 0)
      {
        need_y_up = std::max((int)(-endPixel.y()) + 10, need_y_up);
        outside = true;
      }
      if (outside)
      {
        continue;
      }
      m_ChangedRegion.enclose(endPixel.x(), endPixel.y());

      if (!ranges[i].free)
      {
        unsigned offset = endPixel.x() + m_metaData.width * endPixel.y();
        if (ranges[i].range < 10)
        {
          m_MapPoints[offset].CurrentChange = ::OCCUPIED;
        }
        else
        {
          m_MapPoints[offset].CurrentChange = ::SAFETY_BORDER;
        }
      }
    }
    lastEndPixel = endPixel;
  }

  // paint free pixels
  for (unsigned i = 0; i < ranges.size(); i++)
  {
    Eigen::Vector2i endPixel = map_pixel[i];

    if (endPixel != lastEndPixel)
    {
      if (endPixel.x() >= m_metaData.width || endPixel.x() < 0 ||
          endPixel.y() >= m_metaData.height || endPixel.y() < 0)
      {
        continue;
      }
      drawLine(sensorPixel, endPixel, ::FREE);
    }
    lastEndPixel = endPixel;
  }

  m_ChangedRegion.clip(
      Box2D<int>(0, 0, m_metaData.width - 1, m_metaData.height - 1));
  m_ExploredRegion.enclose(m_ChangedRegion);
  applyChanges();
  computeOccupancyProbabilities();
  if (need_x_left + need_x_right + need_y_down + need_y_up > 0)
  {
    changeMapSize(need_x_left, need_y_up, need_x_right, need_y_down);
  }
}

double OccupancyMap::contrastFromProbability(int8_t prob)
{
  // range from 0..126 (=127 values) and 128..255 (=128 values)
  double diff = ((double)prob - UNKNOWN);
  double contrast;
  if (prob <= UNKNOWN)
  {
    contrast = (diff / UNKNOWN);  // 0..1
  }
  else
  {
    contrast = (diff / (UNKNOWN + 1));  // 0..1
  }
  return (contrast * contrast);
}

double OccupancyMap::evaluateByContrast()
{
  double contrastSum = 0.0;
  unsigned int contrastCnt = 0;

  for (int y = m_ExploredRegion.minY(); y <= m_ExploredRegion.maxY(); y++)
  {
    for (int x = m_ExploredRegion.minX(); x <= m_ExploredRegion.maxX(); x++)
    {
      int i = x + y * m_metaData.width;
      if (m_MapPoints[i].MeasurementCount > 1)
      {
        int prob = m_MapPoints[i].OccupancyProbability * 100;
        if (prob != NOT_SEEN_YET)  // ignore not yet seen cells
        {
          contrastSum += contrastFromProbability(prob);
          contrastCnt++;
        }
      }
    }
  }
  if ((contrastCnt) > 0)
  {
    return ((contrastSum / contrastCnt) * 100);
  }
  return (0);
}

vector<MeasurePoint>
OccupancyMap::getMeasurePoints(sensor_msgs::LaserScanConstPtr laserData)
{
  vector<MeasurePoint> result;
  result.reserve(laserData->ranges.size());

  double minDist = m_MeasureSamplingStep;

  Point2D lastHitPos;
  Point2D lastUsedHitPos;

  // extract points for measuring
  for (unsigned int i = 0; i < laserData->ranges.size(); i++)
  {
    if (laserData->ranges[i] <= laserData->range_max &&
        laserData->ranges[i] >= laserData->range_min)
    {
      float alpha = laserData->angle_min + i * laserData->angle_increment;
      tf::Vector3 pin;
      tf::Vector3 pout;
      pin.setX(cos(alpha) * laserData->ranges[i]);
      pin.setY(sin(alpha) * laserData->ranges[i]);
      pin.setZ(0);

      pout = m_laserTransform * pin;

      Point2D hitPos(pout.x(), pout.y());

      if (hitPos.distance(lastUsedHitPos) >= minDist)
      {
        MeasurePoint p;
        // preserve borders of segments
        if ((i != 0) && (lastUsedHitPos.distance(lastHitPos) >
                         m_metaData.resolution * 0.5) &&
            (hitPos.distance(lastHitPos) >= minDist * 1.5))
        {
          p.hitPos = lastHitPos;
          p.borderType = RightBorder;
          result.push_back(p);
          p.borderType = LeftBorder;
        }
        else
        {
          p.borderType = NoBorder;
        }
        p.hitPos = hitPos;
        result.push_back(p);
        lastUsedHitPos = hitPos;
      }
      lastHitPos = hitPos;
    }
  }

  // the first and last one are border pixels
  if (result.size() > 0)
  {
    result[0].borderType = LeftBorder;
    result[result.size() - 1].borderType = RightBorder;
  }

  // calculate front check points
  for (unsigned i = 0; i < result.size(); i++)
  {
    CVec2 diff;

    switch (result[i].borderType)
    {
      case NoBorder:
        diff = result[i - 1].hitPos - result[i + 1].hitPos;
        break;
      case LeftBorder:
        diff = result[i].hitPos - result[i + 1].hitPos;
        break;
      case RightBorder:
        diff = result[i - 1].hitPos - result[i].hitPos;
        break;
    }

    CVec2 normal = diff.rotate(Math::Pi * 0.5);
    normal.normalize();
    result[i].normal = normal;
    normal *= m_metaData.resolution * sqrt(2.0) * 10.0;

    result[i].frontPos = result[i].hitPos + normal;
  }

  return result;
}

float OccupancyMap::computeScore(Pose robotPose,
                                 std::vector<MeasurePoint> measurePoints)
{
  // This is a very simple implementation, using only the end point of the
  // beam.
  // For every beam the end cell is computed and tested if the cell is
  // occupied.
  unsigned lastOffset = 0;
  unsigned hitOffset = 0;
  unsigned frontOffset = 0;
  float fittingFactor = 0.0;

  float sinTheta = sin(robotPose.theta());
  float cosTheta = cos(robotPose.theta());

  for (unsigned int i = 0; i < measurePoints.size(); i++)
  {
    // fast variant:
    float x = cosTheta * measurePoints[i].hitPos.x() -
              sinTheta * measurePoints[i].hitPos.y() + robotPose.x();
    float y = sinTheta * measurePoints[i].hitPos.x() +
              cosTheta * measurePoints[i].hitPos.y() + robotPose.y();
    geometry_msgs::Point hitPos;
    hitPos.x = x;
    hitPos.y = y;

    Eigen::Vector2i hitPixel = map_tools::toMapCoords(hitPos, m_metaData.origin,
                                                      m_metaData.resolution);
    hitOffset = hitPixel.x() + m_metaData.width * hitPixel.y();

    // avoid multiple measuring of same pixel or unknown pixel
    if ((hitOffset == lastOffset) || (hitOffset >= unsigned(m_ByteSize)) ||
        (m_MapPoints[hitOffset].MeasurementCount == 0))
    {
      continue;
    }

    if (m_BacksideChecking)
    {
      // avoid matching of back and front pixels of obstacles
      x = cosTheta * measurePoints[i].frontPos.x() -
          sinTheta * measurePoints[i].frontPos.y() + robotPose.x();
      y = sinTheta * measurePoints[i].frontPos.x() +
          cosTheta * measurePoints[i].frontPos.y() + robotPose.y();
      geometry_msgs::Point frontPos;
      frontPos.x = x;
      frontPos.y = y;

      Eigen::Vector2i frontPixel = map_tools::toMapCoords(
          frontPos, m_metaData.origin, m_metaData.resolution);
      frontOffset = frontPixel.x() + m_metaData.width * frontPixel.y();

      if ((frontOffset >= unsigned(m_ByteSize)) ||
          (m_MapPoints[frontOffset].MeasurementCount == 0))
      {
        continue;
      }
    }

    lastOffset = hitOffset;
    fittingFactor += m_MapPoints[hitOffset].OccupancyProbability;
  }
  return fittingFactor;
}

void OccupancyMap::drawLine(Eigen::Vector2i& startPixel,
                            Eigen::Vector2i& endPixel, char value)
{
  // bresenham algorithm
  int xstart = startPixel.x();
  int ystart = startPixel.y();
  int xend = endPixel.x();
  int yend = endPixel.y();

  int x, y, t, dist, xerr, yerr, dx, dy, incx, incy;
  // compute distances
  dx = xend - xstart;
  dy = yend - ystart;

  // compute increment
  if (dx < 0)
  {
    incx = -1;
    dx = -dx;
  }
  else
  {
    incx = dx ? 1 : 0;
  }

  if (dy < 0)
  {
    incy = -1;
    dy = -dy;
  }
  else
  {
    incy = dy ? 1 : 0;
  }

  // which distance is greater?
  dist = (dx > dy) ? dx : dy;
  // initializing
  x = xstart;
  y = ystart;
  xerr = dx;
  yerr = dy;

  // compute cells
  for (t = 0; t < dist; t++)
  {
    int index = x + m_metaData.width * y;
    // set flag to free if no flag is set
    // (do not overwrite occupied cells)
    if (index < 0 || index > m_ByteSize)
    {
      continue;
    }
    if (m_MapPoints[index].CurrentChange == NO_CHANGE)
    {
      m_MapPoints[index].CurrentChange = value;
    }
    if (m_MapPoints[index].CurrentChange == ::OCCUPIED ||
        m_MapPoints[index].CurrentChange == ::SAFETY_BORDER ||
        m_MapPoints[index].CurrentChange == ::CONTRAST)
    {
      return;
    }
    xerr += dx;
    yerr += dy;
    if (xerr > dist)
    {
      xerr -= dist;
      x += incx;
    }
    if (yerr > dist)
    {
      yerr -= dist;
      y += incy;
    }
  }
}

void OccupancyMap::applyChanges()
{
  for (int y = m_ChangedRegion.minY() + 1; y <= m_ChangedRegion.maxY() - 1; y++)
  {
    int yOffset = m_metaData.width * y;
    for (int x = m_ChangedRegion.minX() + 1; x <= m_ChangedRegion.maxX() - 1;
         x++)
    {
      int i = x + yOffset;

      if ((m_MapPoints[i].CurrentChange == ::FREE ||
           m_MapPoints[i].CurrentChange == ::OCCUPIED ||
           m_MapPoints[i].CurrentChange == ::CONTRAST) &&
          m_MapPoints[i].MeasurementCount < SHRT_MAX)
      {
        m_MapPoints[i].MeasurementCount++;
      }
      if (m_MapPoints[i].CurrentChange == ::OCCUPIED &&
          m_MapPoints[i].OccupancyCount < SHRT_MAX)
      {
        m_MapPoints[i].OccupancyCount += 4;
      }
    }
  }
  for (int y = m_ChangedRegion.minY() + 1; y <= m_ChangedRegion.maxY() - 1; y++)
  {
    int yOffset = m_metaData.width * y;
    for (int x = m_ChangedRegion.minX() + 1; x <= m_ChangedRegion.maxX() - 1;
         x++)
    {
      int i = x + yOffset;
      if (m_MapPoints[i].OccupancyCount > m_MapPoints[i].MeasurementCount)
        m_MapPoints[i].OccupancyCount = m_MapPoints[i].MeasurementCount;
    }
  }
}

void OccupancyMap::clearChanges()
{
  m_ChangedRegion.expand(2);
  m_ChangedRegion.clip(
      Box2D<int>(0, 0, m_metaData.width - 1, m_metaData.height - 1));
  for (int y = m_ChangedRegion.minY(); y <= m_ChangedRegion.maxY(); y++)
  {
    int yOffset = m_metaData.width * y;
    for (int x = m_ChangedRegion.minX(); x <= m_ChangedRegion.maxX(); x++)
    {
      int i = x + yOffset;
      m_MapPoints[i].CurrentChange = NO_CHANGE;
    }
  }
  m_ChangedRegion =
      Box2D<int>(m_metaData.width - 1, m_metaData.height - 1, 0, 0);
}

void OccupancyMap::incrementMeasurementCount(Eigen::Vector2i p)
{
  if (p.x() >= m_metaData.width || p.y() >= m_metaData.height)
  {
    return;
  }
  unsigned i = p.x() + m_metaData.width * p.y();
  if (m_MapPoints[i].CurrentChange == NO_CHANGE &&
      m_MapPoints[i].MeasurementCount < USHRT_MAX)
  {
    m_MapPoints[i].CurrentChange = ::FREE;
    m_MapPoints[i].MeasurementCount++;
  }
}

void OccupancyMap::incrementOccupancyCount(Eigen::Vector2i p)
{
  if (p.x() >= m_metaData.width || p.y() >= m_metaData.height)
  {
    return;
  }
  unsigned i = p.x() + m_metaData.width * p.y();
  if ((m_MapPoints[i].CurrentChange == NO_CHANGE ||
       m_MapPoints[i].CurrentChange == ::FREE) &&
      m_MapPoints[i].MeasurementCount < USHRT_MAX)
  {
    m_MapPoints[i].CurrentChange = ::OCCUPIED;
    m_MapPoints[i].OccupancyCount++;
  }
}

void OccupancyMap::scaleDownCounts(int maxCount)
{
  clearChanges();
  if (maxCount > 0)
  {
    for (unsigned i = 0; i < m_ByteSize; i++)
    {
      int scalingFactor = m_MapPoints[i].MeasurementCount / maxCount;
      if (scalingFactor != 0)
      {
        m_MapPoints[i].MeasurementCount /= scalingFactor;
        m_MapPoints[i].OccupancyCount /= scalingFactor;
      }
    }
  }
  maximizeChangedRegion();
  applyChanges();
  computeOccupancyProbabilities();
}

void OccupancyMap::markRobotPositionFree()
{
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  geometry_msgs::Point endPosWorld =
      map_tools::transformPoint(point, m_latestMapTransform);
  Eigen::Vector2i robotPixel = map_tools::toMapCoords(
      endPosWorld, m_metaData.origin, m_metaData.resolution);

  int width = 0.25 / m_metaData.resolution;
  for (int i = robotPixel.y() - width; i <= robotPixel.y() + width; i++)
  {
    for (int j = robotPixel.x() - width; j <= robotPixel.x() + width; j++)
    {
      incrementMeasurementCount(Eigen::Vector2i(j, i));
    }
  }
  Box2D<int> robotBox(robotPixel.x() - width, robotPixel.y() - width,
                      robotPixel.x() + width, robotPixel.y() + width);
  m_ChangedRegion.enclose(robotBox);
  m_ExploredRegion.enclose(robotBox);
}

QImage OccupancyMap::getProbabilityQImage(int trancparencyThreshold,
                                          bool showInaccessible) const
{
  QImage retImage(m_metaData.width, m_metaData.height, QImage::Format_RGB32);
  for (int y = 0; y < m_metaData.height; y++)
  {
    for (int x = 0; x < m_metaData.width; x++)
    {
      int i = x + y * m_metaData.width;
      int value = UNKNOWN;
      if (m_MapPoints[i].MeasurementCount > 0)
      {
        value =
            static_cast<int>((1.0 - m_MapPoints[i].OccupancyProbability) * 255);
        if (m_MapPoints[i].MeasurementCount < trancparencyThreshold)
        {
          value = static_cast<int>(
              (0.75 + 0.025 * m_MapPoints[i].MeasurementCount) *
              (1.0 - m_MapPoints[i].OccupancyProbability) * 255);
        }
      }
      retImage.setPixel(x, y, qRgb(value, value, value));
    }
  }
  return retImage;
}

void OccupancyMap::getOccupancyProbabilityImage(vector<int8_t>& data,
                                                nav_msgs::MapMetaData& metaData)
{
  metaData = m_metaData;
  data.resize(m_metaData.width * m_metaData.height);
  std::fill(data.begin(), data.end(),
            (int8_t)NOT_SEEN_YET);  // note: linker error without strange cast
                                    // from int8_t to int8_t
  for (int y = m_ExploredRegion.minY(); y <= m_ExploredRegion.maxY(); y++)
  {
    int yOffset = m_metaData.width * y;
    for (int x = m_ExploredRegion.minX(); x <= m_ExploredRegion.maxX(); x++)
    {
      int i = x + yOffset;
      if (m_MapPoints[i].MeasurementCount < 1)
      {
        continue;
      }
      if (m_MapPoints[i].OccupancyProbability == UNKNOWN_LIKELIHOOD)
      {
        data[i] = NOT_SEEN_YET;
      }
      else
      {
        data[i] = (int)(m_MapPoints[i].OccupancyProbability *
                        99);  // TODO maybe - 2 (or *0.99 or smth)
      }
    }
  }
}

void OccupancyMap::maximizeChangedRegion()
{
  m_ChangedRegion = m_ExploredRegion;
}

void OccupancyMap::applyMasking(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  if (msg->data.size() != m_ByteSize)
  {
    ROS_ERROR_STREAM("Size Mismatch between SLAM map ("
                     << m_ByteSize << ") and masking map (" << msg->data.size()
                     << ")");
    return;
  }
  for (size_t y = 0; y < msg->info.height; y++)
  {
    int yOffset = msg->info.width * y;
    for (size_t x = 0; x < msg->info.width; x++)
    {
      int i = yOffset + x;

      switch (msg->data[i])
      {
        case homer_mapnav_msgs::ModifyMap::BLOCKED:
        case homer_mapnav_msgs::ModifyMap::OBSTACLE:
          // increase measure count of cells which were not yet
          // visible to be able to modify unknown areas
          if (m_MapPoints[i].MeasurementCount == 0)
            m_MapPoints[i].MeasurementCount = 10;

          m_MapPoints[i].OccupancyCount = m_MapPoints[i].MeasurementCount;
          m_MapPoints[i].OccupancyProbability = 1.0;
          m_ExploredRegion.enclose(x, y);
          break;
        case homer_mapnav_msgs::ModifyMap::FREE:
          // see comment above
          if (m_MapPoints[i].MeasurementCount == 0)
            m_MapPoints[i].MeasurementCount = 10;

          m_MapPoints[i].OccupancyCount = 0;
          m_MapPoints[i].OccupancyProbability = 0.0;
          m_ExploredRegion.enclose(x, y);
          break;
        case homer_mapnav_msgs::ModifyMap::HIGH_SENSITIV:
          m_MapPoints[i].HighSensitive = 1;
          break;
      }
    }
  }
}
