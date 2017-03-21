#include <homer_mapping/ParticleFilter/HyperSlamFilter.h>

#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#include "ros/ros.h"

using namespace std;

HyperSlamFilter::HyperSlamFilter (int particleFilterNum, int particleNum )
{
  m_ParticleFilterNum = particleFilterNum;
        if ( m_ParticleFilterNum < 1 )
        {
                m_ParticleFilterNum = 1;
        }
  ROS_DEBUG( "Using %d Hyper Particles.", particleFilterNum);

  m_ParticleNum = particleNum;

        m_DoMapping = true;

  m_DeletionThreshold = 0.98;

  for ( unsigned i=0; i < m_ParticleFilterNum; i++ )
  {
    ostringstream stream;
    stream << "SlamFilter " << i;
    SlamFilter *slamFilter = new SlamFilter ( particleNum );
    m_SlamFilters.push_back ( slamFilter );
  }

  m_BestSlamFilter = m_SlamFilters[0];
}

HyperSlamFilter::~HyperSlamFilter()
{
  for (unsigned i = 0; i < m_ParticleFilterNum; i++)
  {
    if( m_SlamFilters[i] )
    {
      delete m_SlamFilters[i];
      m_SlamFilters[i] = 0;
    }
  }
}

void HyperSlamFilter::setRotationErrorRotating ( float percent )
{
   for ( unsigned i=0; i < m_SlamFilters.size(); i++ )
   {
     m_SlamFilters[i]->setRotationErrorRotating(percent / 100.0);
   }
}

void HyperSlamFilter::setRotationErrorTranslating ( float degreePerMeter )
{
  for ( unsigned int i=0; i < m_SlamFilters.size(); i++ )
  {
    m_SlamFilters[i]->setRotationErrorTranslating(degreePerMeter / 180.0 * M_PI);
  }
}

void HyperSlamFilter::setTranslationErrorTranslating ( float percent )
{
  for ( unsigned int i=0; i < m_SlamFilters.size(); i++ )
  {
    m_SlamFilters[i]->setTranslationErrorTranslating(percent / 100.0);
  }
}

void HyperSlamFilter::setTranslationErrorRotating ( float mPerDegree )
{
  for ( unsigned int i=0; i < m_SlamFilters.size(); i++ )
  {
    m_SlamFilters[i]->setTranslationErrorRotating( mPerDegree / 180.0 * M_PI );
  }
}

void HyperSlamFilter::setMoveJitterWhileTurning ( float mPerDegree )
{
  for ( unsigned int i=0; i < m_SlamFilters.size(); i++ )
  {
    m_SlamFilters[i]->setMoveJitterWhileTurning( mPerDegree / 180.0 * M_PI );
  }
}

void HyperSlamFilter::resetHigh()
{
  for ( unsigned int i=0; i < m_SlamFilters.size(); i++ )
  {
    m_SlamFilters[i]->resetHigh();
  }
}

void HyperSlamFilter::setMapping ( bool doMapping )
{
    m_DoMapping = doMapping;
}

void HyperSlamFilter:: setOccupancyMap ( OccupancyMap* occupancyMap )
{
  for ( unsigned int i=0; i < m_SlamFilters.size(); i++ )
  {
    m_SlamFilters[i]->setOccupancyMap( occupancyMap );
  }
}

void HyperSlamFilter::setRobotPose ( Pose pose, double scatterVarXY, double scatterVarTheta )
{
  for ( unsigned int i=0; i < m_SlamFilters.size(); i++ )
  {
    m_SlamFilters[i]->setRobotPose(pose, scatterVarXY, scatterVarTheta);
  }
}

void HyperSlamFilter::filter ( Transformation2D trans, sensor_msgs::LaserScanConstPtr laserData)
{
  //call filter methods of all particle filters
  for ( unsigned int i=0; i < m_SlamFilters.size(); i++ )
  {
    bool randOnOff;

	if(m_SlamFilters.size() == 1)
	{
		randOnOff = true;
	}
	else
	{
		//if mapping is on, switch on with 80% probability to introduce some randomness in different particle filters
		randOnOff = (rand() % 100) < 80;
	}
    m_SlamFilters[i]->setMapping( m_DoMapping && randOnOff );
    m_SlamFilters[i]->filter(trans, laserData);
	   	
  }
  if(m_SlamFilters.size() != 1)
  {
	  //determine which map has the best/worst contrast
	  double bestContrast = 0.0;
	  static unsigned int bestFilter = 0;
	  double worstContrast = 100.0;
	  static unsigned int worstFilter = 0;

	  for ( unsigned int i=0; i < m_SlamFilters.size(); i++ )
	  {
		double contrast = m_SlamFilters[i]->evaluateByContrast();
		            {
		                    if( contrast > bestContrast )
		                    {
		                            bestContrast = contrast;
		                            bestFilter = i;
		                    }
		                    if ( contrast < worstContrast )
		                    {
		                            worstContrast = contrast;
		                            worstFilter = i;
		                    }
		            }
	  }

	  // set best filter
	  SlamFilter* lastBestFilter = m_BestSlamFilter;
	  m_BestSlamFilter = m_SlamFilters[bestFilter];

	  if ( m_BestSlamFilter != lastBestFilter )
	  {
		ROS_INFO( "Switched to best filter %d (bestContrast: %f) -- the worst filter is %d (worstContrast: %f)", bestFilter, bestContrast, worstFilter, worstContrast); //TODO
	  }

	  if ( bestFilter != worstFilter )
	  {
		if ( worstContrast < ( bestContrast * m_DeletionThreshold ) )
		{
		  // replace the worst filter by the one with the best contrast
		  delete m_SlamFilters[worstFilter];
		  m_SlamFilters[worstFilter] = new SlamFilter ( * m_SlamFilters [bestFilter] );
		}
	  }
   }
}

SlamFilter* HyperSlamFilter::getBestSlamFilter()
{
  return m_BestSlamFilter;
}

void HyperSlamFilter::setDeletionThreshold(double deletionThreshold)
{
  m_DeletionThreshold = deletionThreshold;
}
