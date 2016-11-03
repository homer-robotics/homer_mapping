#ifndef SLAMPARTICLE_H
#define SLAMPARTICLE_H

#include <iostream>
#include <fstream>

#include <homer_mapping/ParticleFilter/Particle.h>

/**
 * @class SlamParticle
 *
 * @author Malte Knauf, Stephan Wirth
 *
 * @brief This class defines a particle for the SlamFilter.
 *
 * This particle contains a weight (inherited from base class) and a Pose (position + orientation).
 * The Pose describes a possible position and orientation of the robot.
 *
 * @see SlamFilter
 * @see Particle
 */
class SlamParticle : public Particle
{

  public:
    /**
     * This constructor assigns the given weight to the member m_Weight.
     * @param weight The weight of the particle.
     * @param robotX X-Position of the robot (world coordinates in m).
     * @param robotY Y-Position of the robot (world coordinates in m).
     * @param robotTheta Orientation of the robot (radiants).
     */
    SlamParticle ( float weight = 1.0, float robotX = 0.0, float robotY = 0.0, float robotTheta = 0.0 );

    ///@brief copy contructor
    SlamParticle ( SlamParticle& slamParticle );

    /**
     * The destructor does nothing so far.
     */
    ~SlamParticle();

    /**
     * Sets the three members m_RobotPositionX, m_RobotPositionY, m_RobotOrientation.
     * @param robotX X-Position of the robot (world coordinates in m).
     * @param robotY Y-Position of the robot (world coordinates in m).
     * @param robotTheta Orientation of the robot (radiants).
     */
    void setRobotPose ( float robotX, float robotY, float robotTheta );

    /**
     * Returns the content of the three members m_RobotPositionX, m_RobotPositionY, m_RobotOrientation.
     * @param[out] robotX X-Position of the robot (world coordinates in m).
     * @param[out] robotY Y-Position of the robot (world coordinates in m).
     * @param[out] robotTheta Orientation of the robot (radiants).
     */
    void getRobotPose ( float& robotX, float& robotY, float& robotTheta );


  private:

    /**
     * These members store the pose of the robot.
     */
    float m_RobotPositionX;
    float m_RobotPositionY;
    float m_RobotOrientation;

};

#endif

