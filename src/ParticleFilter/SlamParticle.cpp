#include <homer_mapping/ParticleFilter/SlamParticle.h>

SlamParticle::SlamParticle(float weight, float robotX, float robotY,
                           float robotTheta)
  : Particle(weight)
{
  m_RobotPositionX = robotX;
  m_RobotPositionY = robotY;
  m_RobotOrientation = robotTheta;
}

SlamParticle::SlamParticle(SlamParticle& slamParticle)
{
  m_RobotPositionX = slamParticle.m_RobotPositionX;
  m_RobotPositionY = slamParticle.m_RobotPositionY;
  m_RobotOrientation = slamParticle.m_RobotOrientation;
}

SlamParticle::~SlamParticle()
{
}

void SlamParticle::setRobotPose(float robotX, float robotY, float robotTheta)
{
  m_RobotPositionX = robotX;
  m_RobotPositionY = robotY;
  m_RobotOrientation = robotTheta;
}

void SlamParticle::getRobotPose(float& robotX, float& robotY, float& robotTheta)
{
  robotX = m_RobotPositionX;
  robotY = m_RobotPositionY;
  robotTheta = m_RobotOrientation;
}
