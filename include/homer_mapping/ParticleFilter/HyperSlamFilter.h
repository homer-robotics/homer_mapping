#ifndef HYPERSLAMFILTER_H
#define HYPERSLAMFILTER_H

#include <homer_mapping/OccupancyMap/OccupancyMap.h>
#include <homer_mapping/ParticleFilter/ParticleFilter.h>
#include <homer_mapping/ParticleFilter/SlamFilter.h>
#include <homer_mapping/ParticleFilter/SlamParticle.h>
#include <homer_nav_libs/Math/Pose.h>
#include <vector>

#include <sensor_msgs/LaserScan.h>

class OccupancyMap;

/**
 * @class HyperSlamFilter
 *
 * @author Malte Knauf, Stephan Wirth, Susanne Maur
 *
 * @brief This class is used to determine the robot's most likely pose with
 * given map and given laser data.
 *
 * A particle filter is a descrete method to describe and compute with a
 * probability distribution.
 * This particle filter uses an occupancy map to determine the probability of
 * robot states.
 * The robot states are stored in a particle together with their weight @see
 * SlamParticle.
 *
 * @see SlamParticle
 * @see ParticleFilter
 * @see OccupancyMap
 */
class HyperSlamFilter
{
public:
  /**
    * This constructor initializes the random number generator and sets the
   * member variables to the given values.
    * @param particleNum Number of particleFilters to use.
    */
  HyperSlamFilter(int particleFilterNum, int particleNum);

  /**
   * The destructor releases the OccupancyMap and the particles.
   */
  ~HyperSlamFilter();

  /**
   * This method runs the filter routine.
   * The given odometry measurement is used as movement hypothesis, the
   * laserData-vector is used
   * as measurement and is used to weight the particles.
   * @param currentPoseOdometry Odometry data of time t.
   * @param laserData msg containing the laser measurement.
   * @param measurementTime Time stamp of the measurement.
   * @param filterDurationTime Returns the time in ms that the filtering needed
   */
  void filter(Pose currentPoseOdometry,
              sensor_msgs::LaserScanConstPtr laserData,
              ros::Time measurementTime, ros::Duration& filterDuration);

  /**
   * Computes and sets the new value for m_Alpha1.
   * @param percent Rotation error while rotating (see class constructor for
   * details)
   */
  void setRotationErrorRotating(float percent);

  /**
   * Computes and sets the new value for m_Alpha2.
   * @param degreesPerMeter Rotation error while translating (see class
   * constructor for details)
   */
  void setRotationErrorTranslating(float degreesPerMeter);

  /**
   * Computes and sets the new value for m_Alpha3.
   * @param percent Translation error while translating (see class constructor
   * for details)
   */
  void setTranslationErrorTranslating(float percent);

  /**
   * Computes and sets the new value for m_Alpha4.
   * @param  mPerDegree Translation error while rotating (see class constructor
   * for details)
   */
  void setTranslationErrorRotating(float mPerDegree);

  /**
   * Computes and sets the new value for m_Alpha5.
   * @param  mPerDegree Move jitter while turning (see class constructor for
   * details)
   */
  void setMoveJitterWhileTurning(float mPerDegree);

  /**
   * Sets a new minimal size of a cluster of scan points which is considered in
   * scan matching.
   * @param  clusterSize Minimal size of a cluster in mm of scan points which is
   * considered in scan matching.
   */
  void setScanMatchingClusterSize(float clusterSize);

  /**
   * Sets whether the map is updated or just used for self-localization.
   * @param doMapping True if robot shall do mapping, false otherwise.
   */
  void setMapping(bool doMapping);

  /**
   * Deletes the current occupancy map and copies a new one into the system.
   * @param occupancyMap The occupancy map to load into the system (copies are
   * being made)
   */
  void setOccupancyMap(OccupancyMap* occupancyMap);

  /**
   * Sets the robot pose in the current occupancy map.
   * @param Robot pose.
   * @param scatterVariance if not equal to 0 the poses are equally scattered
   * around the pose
   */
  void setRobotPose(Pose pose, double scatterVarXY = 0.0,
                    double scatterVarTheta = 0.0);

  /**
   *Returns the best SlamFilter
   */
  SlamFilter* getBestSlamFilter();

  void resetHigh();

  /**
   * Factor (default 0.98) of the contrast of the best particle.
   * If the contrast of the worst particle is below this threshold
   * it will be replaced by the best particle
   * @param deletionThreshold see above
   */
  void setDeletionThreshold(double deletionThreshold);

  /**
   * applies masking to map of slam filter set in GUI
   * @param msg masking message received from GUI
   */
  void applyMasking(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    for (unsigned i = 0; i < m_ParticleFilterNum; ++i)
    {
      m_SlamFilters[i]->applyMasking(msg);
    }
  }

private:
  /** Used SlamFilters */
  std::vector<SlamFilter*> m_SlamFilters;

  /** Number of SlamFilters */
  unsigned m_ParticleFilterNum;

  /** Number of Particles of SlamFilter */
  unsigned m_ParticleNum;

  /** */
  double m_DeletionThreshold;

  /** Best SLAM Filter */
  SlamFilter* m_BestSlamFilter;

  /** Worst SlamFilter */
  SlamFilter* m_WorstSlamFilter;

  bool m_DoMapping;
};
#endif
