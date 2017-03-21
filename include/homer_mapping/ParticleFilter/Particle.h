#ifndef PARTICLE_H
#define PARTICLE_H

#include <fstream>
#include <iostream>

/**
 * @class Particle
 *
 * @author Malte Knauf, Stephan Wirth
 *
 * @brief This class is an implementation of a "particle".
 *
 * A particle as it is used in particle filters is a set of one state and one
 * importance factor (=weight).
 * A set of Particles is a discrete representation of a probability
 * distribution.
 *
 * @see ParticleFilter
 */
class Particle
{
public:
  /**
   * This constructor assigns the given weight to the member m_Weight.
   * @param weight The weight of the particle.
   */
  Particle(float weight = 0.0, int id = 0);

  /**
   * The destructor does nothing so far.
   */
  virtual ~Particle();

  /**
   * This method returns the importance factor of the particle.
   * @return The importance factor (=weight) of the particle.
   */
  inline float getWeight() const
  {
    return m_Weight;
  }

  /**
   * Method to set the weight of the particle.
   * @param newWeight New weight for the particle.
   */
  inline void setWeight(float newWeight)
  {
    m_Weight = newWeight;
  }

  /**
   * @return id of the particle that is stored in m_Id
   */
  inline int getId()
  {
    return m_Id;
  }

private:
  /**
   * Stores the importance factor (=weight) of the particle. This should be a
   * value between 0 and 1.
   */
  float m_Weight;

  /**
   * Stores the id of the particle (for testing purpose)
   */
  int m_Id;
};

#endif
