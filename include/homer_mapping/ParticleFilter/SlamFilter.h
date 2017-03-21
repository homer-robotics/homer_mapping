#ifndef SLAMFILTER_H
#define SLAMFILTER_H

#include <homer_mapping/OccupancyMap/OccupancyMap.h>
#include <homer_mapping/ParticleFilter/ParticleFilter.h>
#include <homer_mapping/ParticleFilter/SlamParticle.h>
#include <homer_nav_libs/Math/Pose.h>
#include <vector>

#include <homer_nav_libs/Math/Math.h>
#include <homer_nav_libs/Math/Transformation2D.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_broadcaster.h>

#include <cmath>
#include <fstream>
#include <sstream>

#include "ros/ros.h"

class OccupancyMap;

/**
 * @class SlamFilter
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
class SlamFilter : public ParticleFilter<SlamParticle>
{
public:
  /**
    * This constructor initializes the random number generator and sets the
   * member variables to the given values.
    * @param particleNum Number of particles to use.
    */
  SlamFilter(int particleNum);

  /// @brief copy constructor
  SlamFilter(SlamFilter& slamFilter);

  /**
   * The destructor releases the OccupancyMap and the particles.
   */
  ~SlamFilter();

  /**
   * This method runs the filter routine.
   * The given odometry measurement is used as movement hypothesis, the
   * laserData-vector is used
   * as measurement and is used to weight the particles.
   * @param currentPoseOdometry Odometry data of time t.
   * @param laserData msg containing the laser measurement.
   */
  void filter(Transformation2D trans, sensor_msgs::LaserScanConstPtr laserData);

  /**
   * @return The Pose of the most important particle (particle with highest
   * weight).
   */
  Pose getLikeliestPose();

  /**
   * This method can be used to retrieve the most likely map that is stored by
   * the particle filter.
   * @return Pointer to the most likely occupancy map.
   */
  OccupancyMap* getLikeliestMap() const;

  void resetHigh();

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
   * @param  mPerDegree Translation error while rotating (see class
   * constructor for details)
   */
  void setTranslationErrorRotating(float mPerDegree);

  /**
   * Computes and sets the new value for m_Alpha5.
   * @param  mPerDegree Move jitter while turning (see class constructor for
   * details)
   */
  void setMoveJitterWhileTurning(float mPerDegree);

  /**
   * Sets whether the map is updated or just used for self-localization.
   * @param doMapping True if robot shall do mapping, false otherwise.
   */
  void setMapping(bool doMapping);

  /**
   * Deletes the current occupancy map and copies a new one into the system.
   * @param occupancyMap The occupancy map to load into the system (is being
   * copied)
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
   * @return Vector of current particle poses. The vector is sorted according
   * to the weights of the
   * particles. The pose of the particle with the highest value is the first
   * element of the vector.
   */
  std::vector<Pose> getParticlePoses() const;

  /**
  * @return vector of all particles
  */
  std::vector<SlamParticle*>* getParticles() const;

  /**
   * @return Vector of current particle weights. The vector is sorted by
   * weight, highest weight first.
   */
  std::vector<float> getParticleWeights() const;

  /**
   * Calculates and returns the variance of the current likeliest particle
   * poses.
   * The orientation of the particle is neglected.
   * @param The number of treated particles.
   * @param[out] poseVarianceX The variance of particle poses in x direction.
   * @param[out] poseVarianceY The variance of particle poses in y direction.
   * @param[out] poseVarianceT The variance of particle poses in T rotation.
   */
  void getPoseVariances(int particleNum, float& poseVarianceX,
                        float& poseVarianceY, float& poseVarianceT);

  /**
   * This method reduces the number of particles used in this SlamFilter to
   * the given value.
   * @param newParticleNumber The new number of particles
   */
  void reduceParticleNumber(int newParticleNumber);

  /**
   * This method returns the contrast of the occupancy grid
   * @return Contrast value from 0 (no contrast) to 1 (max. contrast) of the
   * map
   */
  double evaluateByContrast();

  /**
   * This method passes a masking map to to the underlying occupancy map
   */
  void applyMasking(const nav_msgs::OccupancyGrid::ConstPtr& msg);

private:
  /**
   * This method filter outliers in the given laser scan
   * @param rawData the laser scan to check
   * @param maxDiff maximal difference between two adjacent ranges
   * @return filtered scan without outliers
   */
  vector<float> filterOutliers(sensor_msgs::LaserScanConstPtr rawData,
                               float maxDiff);

  /**
   * This method generates Gauss-distributed random variables with the given
   * variance. The computation
   * method is the Polar Method that is described in the book "A First Course
   * on Probability" by Sheldon Ross.
   * @param variance The variance for the Gauss-distribution that is used to
   * generate the random variable.
   * @return A random variable that is N(0, variance) distributed.
   */
  double randomGauss(float variance = 1.0) const;

  /**
   * This method drifts the particles according to the last two odometry
   * readings (time t-1 and time t).
   */
  void drift(Transformation2D odoTrans);

  /**
   * This method weightens each particle according to the given laser
   * measurement in m_LaserData.
   */
  void measure(sensor_msgs::LaserScanPtr laserData);

  /**
   * For weightening the particles, the filter needs a map.
   * This variable holds a pointer to a map.
   * @see OccupancyMap
   */
  OccupancyMap* m_OccupancyMap;

  /**
   * This variable holds the rotation error that the robot makes while it is
   * rotating.
   * Has to be given in percent. Example: robot makes errors of 3 degrees
   * while making a 60 degrees
   * move -> error is 5% -> rotationErrorRotating = 5)
   */
  float m_Alpha1;

  /**
   * This variable holds the rotation error that the robot makes while it is
   * translating
   * (moving forward or backwards). Has to be given in degrees per meter.
   */
  float m_Alpha2;

  /**
   * This variable holds the translation error that the robot makes while it
   * is translating.
   * Has to be given in percent.
   */
  float m_Alpha3;

  /**
   * This variable holds the translation error that the robot makes while it
   * is rotating.
   * This error only carries weight, if a translation es performed at the same
   * time.
   * See also m_Alpha5.
   * Has to be given in milimeters per degree. Example: Robot makes a turn of
   * 10 degrees and moves its
   * center unintentional 15 mm. -> translationErrorRotating = 15.0 / 10.0 =
   * 1.5
   */
  float m_Alpha4;

  /**
   * This variable holds a move jitter that is considered if the robot is
   * turning.
   * Has to be given in milimeters per degree.
   */
  float m_Alpha5;

  /**
   * True if it is the first run of SlamFilter, false otherwise.
   */
  bool m_FirstRun;

  /**
   * This variabe is true, if the SlamFilter is used for mapping and updates
   * the map,
   * false if it is just used for self-localization.
   */
  bool m_DoMapping;

  /**
   *  Time stamp of the last particle filter step
   */

  ros::Time m_LastMoveTime;

  /**
   * Calculates the square of given input f
   * @param f input
   * @return square of input
   */
  template <class T>
  T sqr(T f)
  {
    return f * f;
  }
};
#endif
