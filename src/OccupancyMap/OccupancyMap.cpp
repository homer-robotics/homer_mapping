#include <homer_mapping/OccupancyMap/OccupancyMap.h>

#include <homer_nav_libs/Math/Math.h>

#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <QImage>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <homer_mapnav_msgs/ModifyMap.h>
#include <homer_nav_libs/tools.h>

//uncomment this to get extended information on the tracer
//#define TRACER_OUTPUT

using namespace std;

const float UNKNOWN_LIKELIHOOD = 0.3;

// Flags of current changes //
const char NO_CHANGE = 0;
const char OCCUPIED = 1;
const char FREE = 2;
//the safety border around occupied pixels which is left unchanged
const char SAFETY_BORDER = 3;
///////////////////////////////

//assumed laser measure count for loaded maps
const int LOADED_MEASURECOUNT = 10;


OccupancyMap::OccupancyMap()
{
  initMembers();
}

OccupancyMap::OccupancyMap(float *&occupancyProbability, geometry_msgs::Pose origin, float resolution, int pixelSize, Box2D<int> exploredRegion)
{
    initMembers();


    m_Origin = origin;
    m_Resolution = resolution;
    m_PixelSize = pixelSize;
    m_ByteSize = pixelSize * pixelSize;
    m_ExploredRegion = exploredRegion;
    m_ChangedRegion = exploredRegion;

    if ( m_OccupancyProbability )
    {
      delete[] m_OccupancyProbability;
    }
    m_OccupancyProbability = occupancyProbability;
    for(unsigned i = 0; i < m_ByteSize; i++)
    {
        if(m_OccupancyProbability[i] != 0.5)
        {
            m_MeasurementCount[i] = LOADED_MEASURECOUNT;
            m_OccupancyCount[i] = m_OccupancyProbability[i] * (float)LOADED_MEASURECOUNT;
        }
    }
}


OccupancyMap::OccupancyMap ( const OccupancyMap& occupancyMap )
{
  m_OccupancyProbability = 0;
  m_MeasurementCount = 0;
  m_OccupancyCount = 0;
  m_CurrentChanges = 0;
  m_InaccessibleCount = 0;
  m_HighSensitive = 0;
  m_LaserMaxRange = 0;
  m_LaserMinRange = 0;

  *this = occupancyMap;
}

OccupancyMap::~OccupancyMap()
{
  cleanUp();
}

void OccupancyMap::initMembers()
{
  float mapSize;
  ros::param::get("/homer_mapping/size", mapSize);
  ros::param::get("/homer_mapping/resolution", m_Resolution);
  ros::param::param("/homer_mapping/max_laser", m_LaserMaxRange, (float) 8.0);

  //add one safety pixel
  m_PixelSize = mapSize / m_Resolution + 1;
  m_ByteSize = m_PixelSize * m_PixelSize;

  m_Origin.position.x = -m_PixelSize*m_Resolution/2;
  m_Origin.position.y = -m_PixelSize*m_Resolution/2;
  m_Origin.orientation.w = 1.0;
  m_Origin.orientation.x = 0.0;
  m_Origin.orientation.y = 0.0;
  m_Origin.orientation.z = 0.0;

  ros::param::get("/homer_mapping/backside_checking", m_BacksideChecking);
  ros::param::get("/homer_mapping/obstacle_borders", m_ObstacleBorders);
  ros::param::get("/homer_mapping/measure_sampling_step", m_MeasureSamplingStep);
  ros::param::get("/homer_mapping/laser_scanner/free_reading_distance", m_FreeReadingDistance);

  m_OccupancyProbability = new float[m_ByteSize];
  m_MeasurementCount = new unsigned short[m_ByteSize];
  m_OccupancyCount = new unsigned short[m_ByteSize];
  m_CurrentChanges = new unsigned char[m_ByteSize];
  m_InaccessibleCount = new unsigned char[m_ByteSize];
  m_HighSensitive = new unsigned short[m_ByteSize];
  for ( unsigned i=0; i<m_ByteSize; i++ )
  {
    m_OccupancyProbability[i]=UNKNOWN_LIKELIHOOD;
    m_OccupancyCount[i]=0;
    m_MeasurementCount[i]=0;
    m_CurrentChanges[i]=NO_CHANGE;
    m_InaccessibleCount[i]=0;
    m_HighSensitive[i] = 0;
  }

  m_ExploredRegion=Box2D<int> ( m_PixelSize/2.1, m_PixelSize/2.1, m_PixelSize/1.9, m_PixelSize/1.9 );
  maximizeChangedRegion();
  
  try
  {
  	bool got_transform = m_tfListener.waitForTransform("/base_link","/laser", ros::Time(0), ros::Duration(1));
  	while(!got_transform); 	
  	{
  		ROS_ERROR_STREAM("need transformation from base_link to laser!");
  		got_transform = m_tfListener.waitForTransform("/base_link","/laser", ros::Time(0), ros::Duration(1));
  	}
  	
    m_tfListener.lookupTransform("/base_link", "/laser", ros::Time(0), m_laserTransform);
  }
  catch (tf::TransformException ex) {
      ROS_ERROR_STREAM(ex.what());
  }
  
}


OccupancyMap& OccupancyMap::operator= ( const OccupancyMap & occupancyMap )
{
  // free allocated memory
  cleanUp();

  m_Resolution = occupancyMap.m_Resolution;
  m_ExploredRegion =  occupancyMap.m_ExploredRegion;
  m_PixelSize = occupancyMap.m_PixelSize;
  m_ByteSize = occupancyMap.m_ByteSize;

  ros::param::get("/homer_mapping/backside_checking", m_BacksideChecking);

  // re-allocate all arrays
  m_OccupancyProbability = new float[m_ByteSize];
  m_MeasurementCount = new unsigned short[m_ByteSize];
  m_OccupancyCount = new unsigned short[m_ByteSize];
  m_CurrentChanges = new unsigned char[m_ByteSize];
  m_InaccessibleCount = new unsigned char[m_ByteSize];
  m_HighSensitive = new unsigned short[m_ByteSize];

  // copy array data
  memcpy ( m_OccupancyProbability, occupancyMap.m_OccupancyProbability, m_ByteSize * sizeof ( *m_OccupancyProbability ) );
  memcpy ( m_MeasurementCount, occupancyMap.m_MeasurementCount, m_ByteSize * sizeof ( *m_MeasurementCount ) );
  memcpy ( m_OccupancyCount, occupancyMap.m_OccupancyCount, m_ByteSize * sizeof ( *m_OccupancyCount ) );
  memcpy ( m_CurrentChanges, occupancyMap.m_CurrentChanges, m_ByteSize * sizeof ( *m_CurrentChanges ) );
  memcpy ( m_InaccessibleCount, occupancyMap.m_InaccessibleCount, m_ByteSize * sizeof ( *m_InaccessibleCount ) );
  memcpy ( m_HighSensitive, occupancyMap.m_HighSensitive, m_ByteSize * sizeof ( *m_HighSensitive) );


  return *this;
}

int OccupancyMap::width() const
{
  return m_PixelSize;
}

int OccupancyMap::height() const
{
  return m_PixelSize;
}

float OccupancyMap::getOccupancyProbability ( Eigen::Vector2i p )
{
  unsigned offset = m_PixelSize * p.y() + p.x();
  if ( offset > unsigned ( m_ByteSize ) )
  {
    return UNKNOWN_LIKELIHOOD;
  }
  return m_OccupancyProbability[ offset ];
}

void OccupancyMap::resetHighSensitive()
{
	ROS_INFO_STREAM("High sensitive Areas reseted");
	m_reset_high = true;
}

void OccupancyMap::computeOccupancyProbabilities()
{
  for ( int y = m_ChangedRegion.minY(); y <= m_ChangedRegion.maxY(); y++ )
  {
    int yOffset = m_PixelSize * y;
    for ( int x = m_ChangedRegion.minX(); x <= m_ChangedRegion.maxX(); x++ )
    {
      int i = x + yOffset;
      if ( m_MeasurementCount[i] > 0 )
      {
        m_OccupancyProbability[i] = m_OccupancyCount[i] / static_cast<float>  	( m_MeasurementCount[i] );
		if (m_HighSensitive[i] == 1)
		{
		  if(m_reset_high == true)
		  {
		  	m_OccupancyCount[i] = 0;
		  	m_OccupancyProbability[i] = 0;
		  }
		  if(m_MeasurementCount[i] > 20 )
		  {
		  	m_MeasurementCount[i] = 10; 	
		  	m_OccupancyCount[i] = 10 * m_OccupancyProbability[i];
		  }
		  if(m_OccupancyProbability[i] > 0.3)
		  {
		    m_OccupancyProbability[i] =  1 ; 
		  }
		}      
      }
      else
      {
        m_OccupancyProbability[i] = UNKNOWN_LIKELIHOOD;
      }
    }
  }
  if(m_reset_high  == true)
  {
  	m_reset_high = false;
  }
}

void OccupancyMap::insertLaserData ( sensor_msgs::LaserScan::ConstPtr laserData, tf::Transform transform)
{
  m_latestMapTransform = transform;
  markRobotPositionFree();
  ros::Time stamp = laserData->header.stamp;

  //m_LaserMaxRange = laserData->range_max;
  m_LaserMinRange = laserData->range_min;


  m_LaserPos.x = m_laserTransform.getOrigin().getX();
  m_LaserPos.y = m_laserTransform.getOrigin().getY();

  std::vector<RangeMeasurement> ranges;
  ranges.reserve ( laserData->ranges.size() );

  bool errorFound=false;
  int lastValidIndex=-1;
  float lastValidRange=m_FreeReadingDistance;

  RangeMeasurement rangeMeasurement;
  rangeMeasurement.sensorPos = m_LaserPos;
  
  for ( unsigned int i = 0; i < laserData->ranges.size(); i++ )
  {
    if ( ( laserData->ranges[i] >= m_LaserMinRange ) && ( laserData->ranges[i] <= m_LaserMaxRange ) )
    {
      //if we're at the end of an errorneous segment, interpolate
      //between last valid point and current point
      if ( errorFound )
      {
        //smaller of the two ranges belonging to end points
        float range = Math::min ( lastValidRange, laserData->ranges[i] );
        range -= m_Resolution * 2;

        if ( range < m_FreeReadingDistance )
        {
          range = m_FreeReadingDistance;
        }
        else
          if ( range > m_LaserMaxRange )
          {
            range = m_LaserMaxRange;
          }

        //choose smaller range
        for ( unsigned j=lastValidIndex+1; j<i; j++ )
        {  
			float alpha = laserData->angle_min + j * laserData->angle_increment;
			geometry_msgs::Point point;
			tf::Vector3 pin;
			tf::Vector3 pout;
			pin.setX( cos(alpha) * range);
			pin.setY( sin(alpha) * range);
			pin.setZ(0);

			pout = m_laserTransform * pin;

			point.x = pout.x();
			point.y = pout.y();

			rangeMeasurement.endPos = point;
			rangeMeasurement.free = true;
			ranges.push_back ( rangeMeasurement );
        }                             
      }
	  float alpha = laserData->angle_min + i * laserData->angle_increment;
	  geometry_msgs::Point point;
	  tf::Vector3 pin;
	  tf::Vector3 pout;
	  pin.setX( cos(alpha) * laserData->ranges[i]);
	  pin.setY( sin(alpha) * laserData->ranges[i]);
	  pin.setZ(0);

	  pout = m_laserTransform * pin;

	  point.x = pout.x();
	  point.y = pout.y();

	  rangeMeasurement.endPos = point;
      rangeMeasurement.free = false;
      ranges.push_back ( rangeMeasurement );

      errorFound=false;
      lastValidIndex=i;
      lastValidRange=laserData->ranges[i];
    }
    else
    {
      errorFound=true;
    }
  }

//  if ( errorFound )
//  {
//    for ( unsigned j=lastValidIndex+1; j<laserData->ranges.size(); j++ )
//    {
//      rangeMeasurement.endPos = map_tools::laser_range_to_point(m_FreeReadingDistance, j, laserData->angle_min, laserData->angle_increment, m_tfListener, laserData->header.frame_id, "/base_link"); //

//      rangeMeasurement.free = true;
//      ranges.push_back ( rangeMeasurement );
//    }
//  }

  insertRanges ( ranges , laserData->header.stamp);
}


void OccupancyMap::insertRanges ( vector<RangeMeasurement> ranges, ros::Time stamp )
{
  clearChanges();

  Eigen::Vector2i lastEndPixel;

  //paint safety borders
  if ( m_ObstacleBorders )
  {
    for ( unsigned i=0; i<ranges.size(); i++ )
    {
      geometry_msgs::Point endPosWorld = map_tools::transformPoint(ranges[i].endPos, m_latestMapTransform);
      Eigen::Vector2i endPixel = map_tools::toMapCoords(endPosWorld, m_Origin, m_Resolution);

      for ( int y=endPixel.y()-1; y <= endPixel.y() +1; y++ )
      {
        for ( int x=endPixel.x()-1; x <= endPixel.x() +1; x++ )
        {
          unsigned offset=x+m_PixelSize*y;
          if ( offset < unsigned ( m_ByteSize ) )
          {
            m_CurrentChanges[ offset ] = SAFETY_BORDER;
          }
        }
      }
    }
  }
  //paint safety ranges
  for ( unsigned i=0; i<ranges.size(); i++ )
  {
      geometry_msgs::Point startPosWorld = map_tools::transformPoint(ranges[i].endPos, m_latestMapTransform);
      Eigen::Vector2i startPixel = map_tools::toMapCoords(startPosWorld, m_Origin, m_Resolution);
    geometry_msgs::Point endPos;
    endPos.x = ranges[i].endPos.x * 4;
    endPos.y = ranges[i].endPos.y * 4;

    geometry_msgs::Point endPosWorld = map_tools::transformPoint(endPos, m_latestMapTransform);
    Eigen::Vector2i endPixel = map_tools::toMapCoords(endPosWorld, m_Origin, m_Resolution);


    if(endPixel.x() < 0) endPixel.x() = 0;
    if(endPixel.y() < 0) endPixel.y() = 0;
    if(endPixel.x() >= m_PixelSize) endPixel.x() = m_PixelSize - 1;
    if(endPixel.y() >= m_PixelSize) endPixel.y() = m_PixelSize - 1;

    drawLine ( m_CurrentChanges, startPixel, endPixel, SAFETY_BORDER );
  }

  //paint end pixels
  for ( unsigned i=0; i<ranges.size(); i++ )
  {
    if ( !ranges[i].free )
    {
      geometry_msgs::Point endPosWorld = map_tools::transformPoint(ranges[i].endPos, m_latestMapTransform);
      Eigen::Vector2i endPixel = map_tools::toMapCoords(endPosWorld, m_Origin, m_Resolution);

      if ( endPixel != lastEndPixel )
      {
        unsigned offset = endPixel.x() + m_PixelSize * endPixel.y();
        if ( offset < m_ByteSize )
        {
          m_CurrentChanges[ offset ] = ::OCCUPIED;
        }
      }
      lastEndPixel=endPixel;
      }
  }

  //paint free ranges
  geometry_msgs::Point sensorPosWorld = map_tools::transformPoint(ranges[0].sensorPos, m_latestMapTransform);
  Eigen::Vector2i sensorPixel = map_tools::toMapCoords(sensorPosWorld, m_Origin, m_Resolution);
  
  for ( unsigned i=0; i<ranges.size(); i++ )
  {
    geometry_msgs::Point endPosWorld = map_tools::transformPoint(ranges[i].endPos, m_latestMapTransform);
    Eigen::Vector2i endPixel = map_tools::toMapCoords(endPosWorld, m_Origin, m_Resolution);

    m_ChangedRegion.enclose ( sensorPixel.x(), sensorPixel.y() );
    m_ChangedRegion.enclose ( endPixel.x(), endPixel.y() );

    if ( endPixel != lastEndPixel )
    {
        drawLine ( m_CurrentChanges, sensorPixel, endPixel, ::FREE );
    }

    lastEndPixel=endPixel;
  }

  m_ChangedRegion.clip ( Box2D<int> ( 0,0,m_PixelSize-1,m_PixelSize-1 ) );
  m_ExploredRegion.enclose ( m_ChangedRegion );
  applyChanges();
  computeOccupancyProbabilities();
}

double OccupancyMap::contrastFromProbability ( int8_t prob )
{
  // range from 0..126 (=127 values) and 128..255 (=128 values)
  double diff = ( ( double ) prob - UNKNOWN );
  double contrast;
  if ( prob <= UNKNOWN )
  {
    contrast = ( diff / UNKNOWN ); // 0..1
  }
  else
  {
    contrast = ( diff / ( UNKNOWN+1 ) );  // 0..1
  }
  return ( contrast * contrast );
}

double OccupancyMap::evaluateByContrast()
{
  double contrastSum = 0.0;
  unsigned int contrastCnt = 0;

  for ( int y = m_ExploredRegion.minY(); y <= m_ExploredRegion.maxY(); y++ )
  {
    for ( int x = m_ExploredRegion.minX(); x <= m_ExploredRegion.maxX(); x++ )
    {
      int i = x + y * m_PixelSize;
      if ( m_MeasurementCount [i] > 1 )
      {
        int prob = m_OccupancyProbability[i] * 100;
        if ( prob != NOT_SEEN_YET ) // ignore not yet seen cells
        {
          contrastSum += contrastFromProbability ( prob );
          contrastCnt++;
        }
      }
    }
  }
  if ( ( contrastCnt ) > 0 )
  {
    return ( ( contrastSum / contrastCnt ) * 100 );
  }
  return ( 0 );
}



vector<MeasurePoint> OccupancyMap::getMeasurePoints (sensor_msgs::LaserScanConstPtr laserData)
{
  vector<MeasurePoint> result;
  result.reserve ( laserData->ranges.size() );

  double minDist = m_MeasureSamplingStep;

  //m_LaserMaxRange = laserData->range_max;
  m_LaserMinRange = laserData->range_min;

  Point2D lastHitPos;
  Point2D lastUsedHitPos;

  //extract points for measuring
  for ( unsigned int i=0; i < laserData->ranges.size(); i++ )
  {
    if ( laserData->ranges[i] <= m_LaserMaxRange && laserData->ranges[i] >= m_LaserMinRange )
    {
		float alpha = laserData->angle_min + i * laserData->angle_increment;
		tf::Vector3 pin;
		tf::Vector3 pout;
		pin.setX( cos(alpha) * laserData->ranges[i]);
		pin.setY( sin(alpha) * laserData->ranges[i]);
		pin.setZ(0);

		pout = m_laserTransform * pin;
        
        Point2D hitPos(pout.x(), pout.y());

      if ( hitPos.distance ( lastUsedHitPos ) >= minDist )
      {
        MeasurePoint p;
        //preserve borders of segments
        if ( ( i!=0 ) &&
                ( lastUsedHitPos.distance ( lastHitPos ) > m_Resolution*0.5 ) &&
                ( hitPos.distance ( lastHitPos ) >= minDist*1.5 ) )
        {
          p.hitPos = lastHitPos;
          p.borderType = RightBorder;
          result.push_back ( p );
          p.hitPos = hitPos;
          p.borderType = LeftBorder;
          result.push_back ( p );
          lastUsedHitPos = hitPos;
        }
        else
        {
          //save current point
          p.hitPos = hitPos;
          p.borderType = NoBorder;
          result.push_back ( p );
          lastUsedHitPos = hitPos;
        }
      }
      lastHitPos = hitPos;
    }
  }

  //the first and last one are border pixels
  if ( result.size() > 0 )
  {
    result[0].borderType = LeftBorder;
    result[result.size()-1].borderType = RightBorder;
  }

  //calculate front check points
  for ( unsigned i=0; i < result.size(); i++ )
  {
    CVec2 diff;

    switch ( result[i].borderType )
    {
      case NoBorder:
        diff = result[i-1].hitPos - result[i+1].hitPos;
        break;
      case LeftBorder:
        diff = result[i].hitPos - result[i+1].hitPos;
        break;
      case RightBorder:
        diff = result[i-1].hitPos - result[i].hitPos;
        break;
    }

    CVec2 normal = diff.rotate ( Math::Pi * 0.5 );
    normal.normalize();
    normal *= m_Resolution * sqrt ( 2.0 ) * 10.0;

    result[i].frontPos = result[i].hitPos + normal;
  }

  return result;
}


float OccupancyMap::computeScore ( Pose robotPose, std::vector<MeasurePoint> measurePoints )
{
  // This is a very simple implementation, using only the end point of the beam.
  // For every beam the end cell is computed and tested if the cell is occupied.
  unsigned lastOffset=0;
  unsigned hitOffset=0;
  unsigned frontOffset=0;
  float fittingFactor = 0.0;

  float sinTheta = sin ( robotPose.theta() );
  float cosTheta = cos ( robotPose.theta() );

  for ( unsigned int i = 0; i < measurePoints.size(); i++ )
  {
    //fast variant:
    float x = cosTheta * measurePoints[i].hitPos.x() - sinTheta * measurePoints[i].hitPos.y() + robotPose.x();
    float y = sinTheta * measurePoints[i].hitPos.x() + cosTheta * measurePoints[i].hitPos.y() + robotPose.y();
    geometry_msgs::Point hitPos;
    hitPos.x = x;
    hitPos.y = y;

    Eigen::Vector2i hitPixel = map_tools::toMapCoords(hitPos, m_Origin, m_Resolution);
    hitOffset = hitPixel.x() + m_PixelSize*hitPixel.y();

    //avoid multiple measuring of same pixel or unknown pixel
    if ( ( hitOffset == lastOffset ) || ( hitOffset >= unsigned ( m_ByteSize ) ) || ( m_MeasurementCount[hitOffset] == 0 ) )
    {
      continue;
    }

    if ( m_BacksideChecking )
    {
      //avoid matching of back and front pixels of obstacles
      x = cosTheta * measurePoints[i].frontPos.x() - sinTheta * measurePoints[i].frontPos.y() + robotPose.x();
      y = sinTheta * measurePoints[i].frontPos.x() + cosTheta * measurePoints[i].frontPos.y() + robotPose.y();
      geometry_msgs::Point frontPos;
      frontPos.x = x;
      frontPos.y = y;

      Eigen::Vector2i frontPixel = map_tools::toMapCoords(frontPos, m_Origin, m_Resolution);
      frontOffset = frontPixel.x() + m_PixelSize*frontPixel.y();

      if ( ( frontOffset >= unsigned ( m_ByteSize ) ) || ( m_MeasurementCount[frontOffset] == 0 ) )
      {
        continue;
      }
    }

    lastOffset=hitOffset;
    //fittingFactor += m_SmoothOccupancyProbability[ offset ];
    fittingFactor += m_OccupancyProbability[ hitOffset ];
  }
  return fittingFactor;
}


template<class DataT>
void OccupancyMap::drawLine ( DataT* data, Eigen::Vector2i& startPixel, Eigen::Vector2i& endPixel, char value )
{

  //bresenham algorithm
  int xstart = startPixel.x();
  int ystart = startPixel.y();
  int xend = endPixel.x();
  int yend = endPixel.y();

  int x, y, t, dist, xerr, yerr, dx, dy, incx, incy;
  // compute distances
  dx = xend - xstart;
  dy = yend - ystart;

  // compute increment
  if ( dx < 0 )
  {
    incx = -1;
    dx = -dx;
  }
  else
  {
    incx = dx ? 1 : 0;
  }

  if ( dy < 0 )
  {
    incy = -1;
    dy = -dy;
  }
  else
  {
    incy = dy ? 1 : 0;
  }

  // which distance is greater?
  dist = ( dx > dy ) ? dx : dy;
  // initializing
  x = xstart;
  y = ystart;
  xerr = dx;
  yerr = dy;

  // compute cells
  for ( t = 0; t < dist; t++ )
  {
    int index = x + m_PixelSize * y;
    // set flag to free if no flag is set
    // (do not overwrite occupied cells)
    if(index < 0) continue;
    if ( data[index] == NO_CHANGE )
    {
      data[index] = value;
    }
/*    if ( data[index] == OCCUPIED || data[index] == SAFETY_BORDER )
    {
      return;
    }*/
    xerr += dx;
    yerr += dy;
    if ( xerr > dist )
    {
      xerr -= dist;
      x += incx;
    }
    if ( yerr > dist )
    {
      yerr -= dist;
      y += incy;
    }
  }
}


void OccupancyMap::applyChanges()
{
  for ( int y = m_ChangedRegion.minY(); y <= m_ChangedRegion.maxY(); y++ )
  {
    int yOffset = m_PixelSize * y;
    for ( int x = m_ChangedRegion.minX(); x <= m_ChangedRegion.maxX(); x++ )
    {
      int i = x + yOffset;
      if ( ( m_CurrentChanges[i] == ::FREE || m_CurrentChanges[i] == ::OCCUPIED ) && m_MeasurementCount[i] < SHRT_MAX )
      {
        m_MeasurementCount[i]++;
      }
      if ( m_CurrentChanges[i] == ::OCCUPIED && m_OccupancyCount[i] < USHRT_MAX )
      {
        m_OccupancyCount[i]++;
      }
    }
  }
}

void OccupancyMap::clearChanges()
{
  m_ChangedRegion.expand ( 2 );
  m_ChangedRegion.clip ( Box2D<int> ( 0,0,m_PixelSize-1,m_PixelSize-1 ) );
  for ( int y = m_ChangedRegion.minY(); y <= m_ChangedRegion.maxY(); y++ )
  {
    int yOffset = m_PixelSize * y;
    for ( int x = m_ChangedRegion.minX(); x <= m_ChangedRegion.maxX(); x++ )
    {
      int i = x + yOffset;
      m_CurrentChanges[i] = NO_CHANGE;
    }
  }
  m_ChangedRegion=Box2D<int> ( m_PixelSize - 1, m_PixelSize - 1, 0, 0 );
}

void OccupancyMap::incrementMeasurementCount ( Eigen::Vector2i p )
{
  unsigned index = p.x() + m_PixelSize * p.y();
  if ( index < m_ByteSize )
  {
    if ( m_CurrentChanges[index] == NO_CHANGE && m_MeasurementCount[index] < USHRT_MAX )
    {
      m_CurrentChanges[index] = ::FREE;
      m_MeasurementCount[index]++;
    }
  }
  else
  {
    ROS_ERROR( "Index out of bounds: x = %d, y = %d", p.x(), p.y() );
  }
}

void OccupancyMap::incrementOccupancyCount ( Eigen::Vector2i p )
{
  int index = p.x() + m_PixelSize * p.y();
  if ( ( m_CurrentChanges[index] == NO_CHANGE || m_CurrentChanges[index] == ::FREE ) && m_MeasurementCount[index] < USHRT_MAX )
  {
    m_CurrentChanges[index] = ::OCCUPIED;
    m_OccupancyCount[index]++;
  }
}

void OccupancyMap::scaleDownCounts ( int maxCount )
{
  clearChanges();
  if ( maxCount <= 0 )
  {
    ROS_WARN("WARNING: argument maxCount is choosen to small, resetting map.");
    memset ( m_MeasurementCount, 0, m_ByteSize );
    memset ( m_OccupancyCount, 0, m_ByteSize );
    memset ( m_InaccessibleCount, 0, m_ByteSize );
  }
  else
  {
    for ( unsigned i = 0; i < m_ByteSize; i++ )
    {
      int scalingFactor = m_MeasurementCount[i] / maxCount;
      if ( scalingFactor != 0 )
      {
        m_MeasurementCount[i] /= scalingFactor;
        m_OccupancyCount[i] /= scalingFactor;
        m_InaccessibleCount[i] /= scalingFactor;
      }
      if ( m_InaccessibleCount[i] > maxCount )
      {
        m_InaccessibleCount[i] = maxCount;
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
  geometry_msgs::Point endPosWorld = map_tools::transformPoint(point, m_latestMapTransform);
  Eigen::Vector2i robotPixel = map_tools::toMapCoords(endPosWorld, m_Origin, m_Resolution);

  int width = 0.4 / m_Resolution;
  for ( int i = robotPixel.y() - width; i <= robotPixel.y() + width; i++ )
  {
    for ( int j = robotPixel.x() - width; j <= robotPixel.x() + width; j++ )
    {
      incrementMeasurementCount ( Eigen::Vector2i ( i, j ) );
    }
  }
  Box2D<int> robotBox ( robotPixel.x()-width, robotPixel.y()-width, robotPixel.x() +width, robotPixel.y() +width );
  m_ChangedRegion.enclose ( robotBox );
  m_ExploredRegion.enclose ( robotBox );
}


QImage OccupancyMap::getProbabilityQImage ( int trancparencyThreshold, bool showInaccessible ) const
{
  QImage retImage ( m_PixelSize, m_PixelSize, QImage::Format_RGB32 );
  for ( int y = 0; y < m_PixelSize; y++ )
  {
    for ( int x = 0; x < m_PixelSize; x++ )
    {
      int index = x + y * m_PixelSize;
      int value = UNKNOWN;
      if ( m_MeasurementCount[index] > 0 )
      {
        value = static_cast<int> ( ( 1.0 - m_OccupancyProbability[index] ) * 255 );
        if ( m_MeasurementCount[index] < trancparencyThreshold )
        {
          value = static_cast<int> ( ( 0.75 + 0.025 * m_MeasurementCount[index] ) * ( 1.0 - m_OccupancyProbability[index] ) * 255 );
        }
      }
      if ( showInaccessible && m_InaccessibleCount[index] >= 2 )
      {
        value = 0;
      }
      retImage.setPixel ( x, y, qRgb ( value, value, value ) );
    }
  }
  return retImage;
}

void OccupancyMap::getOccupancyProbabilityImage ( vector<int8_t>& data, int& width, int& height, float& resolution )
{
  width = m_PixelSize;
  height = m_PixelSize;
  resolution = m_Resolution;
  data.resize(m_PixelSize * m_PixelSize);
  std::fill(data.begin(), data.end(), (int8_t)NOT_SEEN_YET); //note: linker error without strange cast from int8_t to int8_t
  for ( int y = m_ExploredRegion.minY(); y <= m_ExploredRegion.maxY(); y++ )
  {
    int yOffset = m_PixelSize * y;
    for ( int x = m_ExploredRegion.minX(); x <= m_ExploredRegion.maxX(); x++ )
    {
      int i = x + yOffset;
      if ( m_MeasurementCount[i] < 1 )
      {
        continue;
      }
      // set inaccessible points to black
      if ( m_InaccessibleCount[i] >= 2 )
      {
        data[i] = 99;
        continue;
      }
      if(m_OccupancyProbability[i] == UNKNOWN_LIKELIHOOD)
      {
          data[i] = NOT_SEEN_YET;
      }
      else
      {
        data[i] = (int)(m_OccupancyProbability[i] * 99); //TODO maybe - 2 (or *0.99 or smth)
      }
    }
  }
}

void OccupancyMap::maximizeChangedRegion()
{
  m_ChangedRegion=m_ExploredRegion;
}

void OccupancyMap::applyMasking(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(msg->data.size() != m_ByteSize)
    {
        ROS_ERROR_STREAM("Size Mismatch between SLAM map (" << m_ByteSize << ") and masking map (" << msg->data.size() << ")");
        return;
    }
    for(size_t y = 0; y < msg->info.height; y++)
    {
        int yOffset = msg->info.width * y;
        for(size_t x = 0; x < msg->info.width; x++)
        {
            int i = yOffset + x;

            switch(msg->data[i])
            {
            case homer_mapnav_msgs::ModifyMap::BLOCKED:
            case homer_mapnav_msgs::ModifyMap::OBSTACLE:
                //increase measure count of cells which were not yet visible to be able to modify unknown areas
                if(m_MeasurementCount[i] == 0)
                    m_MeasurementCount[i] = 10;

                m_OccupancyCount[i] = m_MeasurementCount[i];
                m_OccupancyProbability[i] = 1.0;
                m_ExploredRegion.enclose(x, y);
                break;
            case homer_mapnav_msgs::ModifyMap::FREE:
                //see comment above
                if(m_MeasurementCount[i] == 0)
                    m_MeasurementCount[i] = 10;

                m_OccupancyCount[i] = 0;
                m_OccupancyProbability[i] = 0.0;
                m_ExploredRegion.enclose(x, y);
                break;
            case homer_mapnav_msgs::ModifyMap::HIGH_SENSITIV:
            	m_HighSensitive[i] = 1;
                break;
            }
        }
    }
}

void OccupancyMap::cleanUp()
{
  if ( m_OccupancyProbability )
  {
    delete[] m_OccupancyProbability;
  }
  if ( m_MeasurementCount )
  {
    delete[] m_MeasurementCount;
  }
  if ( m_OccupancyCount )
  {
    delete[] m_OccupancyCount;
  }
  if ( m_CurrentChanges )
  {
    delete[] m_CurrentChanges;
  }
  if ( m_InaccessibleCount )
  {
    delete[] m_InaccessibleCount;
  }
  if ( m_HighSensitive ) 
  {
    delete[] m_HighSensitive;
  }
}
