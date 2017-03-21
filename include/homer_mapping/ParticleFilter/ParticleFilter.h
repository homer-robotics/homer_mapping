#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <limits.h>
#include <omp.h>
#include <cmath>
#include <iostream>

#include <ros/ros.h>

class Particle;

const float MIN_EFFECTIVE_PARTICLE_WEIGHT = 0.2;

/**
 * @class ParticleFilter
 *
 * @author Malte Knauf, Stephan Wirth
 *
 * @brief This class is a template class for a particle filter.
 *
 * A particle filter is a descrete method to describe and compute with a
 * probability distribution.
 * This template class implements the basic methods for a particle filter:
 * sort() and resample().
 * Use this class do derivate your custom particle filter from it. Use a
 * self-defined subclass of
 * Particle as ParticleType.
 *
 * @see Particle
 */
template <class ParticleType>
class ParticleFilter
{
public:
  /**
   * The constructor initializes the random number generator and allocates the
   * memory for the particle lists.
   * The lists will have particleNum elements.
   * @param particleNum Number of particles for the filter.
   */
  ParticleFilter<ParticleType>(int particleNum);

  /**
   * The destructor releases the particle lists.
   */
  virtual ~ParticleFilter();

  /**
   * @return Number of particles used in this filter
   */
  int getParticleNum();

  /**
   * @return The number of effective particles (according to "Improving
   * Grid-based SLAM with Rao-Blackwellized Particle
   * Filters by Adaptive Proposals and Selective Resampling (2005)" by Giorgio
   * Grisetti, Cyrill Stachniss, Wolfram Burgard
   */
  int getEffectiveParticleNum() const;
  int getEffectiveParticleNum2() const;

  /**
   * @return Pointer to the particle that has the highest weight.
   */
  ParticleType* getBestParticle() const;

protected:
  /**
   * This method generates a random variable in the interval [0,1].
   * @param init The initial value for the static random base number. When
   * running the constructor of this
   * class, this method is run once with the C-function time() as parameter to
   * initialize it.
   * Then you should use it without parameter.
   * @return Random value between 0 and 1
   */
  double random01(unsigned long init = 0) const;

  /**
   * This method sorts the particles in m_CurrentList from leftIndex to
   * rightIndex according to their weight.
   * The particle with the highest weight is at position 0 after calling this
   * function. The algorithm used here is
   * known as quicksort and works recursively.
   * @param leftIndex Left index of area to sort
   * @param rightIndex Right index of area to sort
   */
  void sort(int leftIndex, int rightIndex);

  /**
   * This method normalizes the weights of the particles. After calling this
   * function, the sum of the weights of
   * all particles in m_CurrentList equals 1.0.
   * In this function the sum of all weights of the particles of m_CurrentList
   * is computed and each weight of each
   * particle is devided through this sum.
   */
  void normalize();

  /**
   * This method selects a new set of particles out of an old set according to
   * their weight
   * (importance resampling). The particles from the list m_CurrentList points
   * to are used as source,
   * m_LastList points to the destination list. The pointers m_CurrentList and
   * m_LastList are switched.
   * The higher the weight of a particle, the more particles are drawn (copied)
   * from this particle.
   * The weight remains untouched, because measure() will be called afterwards.
   * This method only works on a sorted m_CurrentList, therefore sort() is
   * called first.
   */
  void resample();

  /**
   * This method drifts the particles (second step of a filter process).
   * Has to be implemented in sub-classes (pure virtual function).
   */
  virtual void drift() = 0;

  /**
   * This method has to be implemented in sub-classes. It is used to determine
   * the weight of each particle.
   */
  virtual void measure() = 0;

  /**
   * These two pointers point to m_ParticleListOne and to m_ParticleListTwo.
   * The particles are drawn from m_LastList to m_CurrentList to avoid new and
   * delete commands.
   * In each run, the pointers are switched in resample().
   */
  ParticleType** m_CurrentList;
  ParticleType** m_LastList;

  /**
   * Stores the number of particles.
   */
  int m_ParticleNum;

  /**
   * Stores the number of effective particles.
   */
  int m_EffectiveParticleNum;
};

template <class ParticleType>
ParticleFilter<ParticleType>::ParticleFilter(int particleNum)
{
  // initialize particle lists
  m_CurrentList = new ParticleType*[particleNum];
  m_LastList = new ParticleType*[particleNum];

  // initialize random number generator
  random01(time(0));

  m_ParticleNum = particleNum;
}

template <class ParticleType>
ParticleFilter<ParticleType>::~ParticleFilter()
{
  if (m_CurrentList)
  {
    delete[] m_CurrentList;
    m_CurrentList = 0;
  }
  if (m_LastList)
  {
    delete[] m_LastList;
    m_LastList = 0;
  }
}

template <class ParticleType>
int ParticleFilter<ParticleType>::getParticleNum()
{
  return m_ParticleNum;
}

template <class ParticleType>
double ParticleFilter<ParticleType>::random01(unsigned long init) const
{
  static unsigned long n;
  if (init > 0)
  {
    n = init;
  }
  n = 1664525 * n + 1013904223;
  // create double from unsigned long
  return (double)(n / 2) / (double)LONG_MAX;
}

template <class ParticleType>
void ParticleFilter<ParticleType>::sort(int indexLeft, int indexRight)
{
  // SOMETHING LEFT TO SORT?
  if (indexLeft >= indexRight)
  {
    // ready!
    return;
  }

  // CREATE PARTITION
  int le = indexLeft;
  int ri = indexRight;
  int first = le;
  int pivot = ri--;
  while (le <= ri)
  {
    // skip from left
    while (m_CurrentList[le]->getWeight() > m_CurrentList[pivot]->getWeight())
    {
      le++;
    }
    // skip from right
    while ((ri >= first) && (m_CurrentList[ri]->getWeight() <=
                             m_CurrentList[pivot]->getWeight()))
    {
      ri--;
    }
    // now we have two elements to swap
    if (le < ri)
    {
      // swap
      ParticleType* temp = m_CurrentList[le];
      m_CurrentList[le] = m_CurrentList[ri];
      m_CurrentList[ri] = temp;
      le++;
    }
  }

  if (le != pivot)
  {
    // swap
    ParticleType* temp = m_CurrentList[le];
    m_CurrentList[le] = m_CurrentList[pivot];
    m_CurrentList[pivot] = temp;
  }

  // sort left side
  sort(indexLeft, le - 1);
  // sort right side
  sort(le + 1, indexRight);
}

template <class ParticleType>
void ParticleFilter<ParticleType>::normalize()
{
  float weightSum = 0.0;
  for (int i = 0; i < m_ParticleNum; i++)
  {
    weightSum += m_CurrentList[i]->getWeight();
  }
  // only normalize if weightSum is big enough to divide
  if (weightSum > 0.000001)
  {
    // calculating parallel on 8 threats
    omp_set_num_threads(8);
    int i = 0;
    // #pragma omp parallel for
    for (i = 0; i < m_ParticleNum; i++)
    {
      float newWeight = m_CurrentList[i]->getWeight() / weightSum;
      m_CurrentList[i]->setWeight(newWeight);
    }
  }
  else
  {
    ROS_WARN_STREAM("Particle weights VERY small: "
                    << weightSum << ". Got " << m_ParticleNum << " particles.");
  }
}

template <class ParticleType>
void ParticleFilter<ParticleType>::resample()
{
  // swap pointers
  ParticleType** help = m_LastList;
  m_LastList = m_CurrentList;
  m_CurrentList = help;
  // now we copy from m_LastList to m_CurrentList

  int drawIndex = 0;
  // index of the particle where we are drawing to
  int targetIndex = 0;

  int numToDraw = 0;
  do
  {
    numToDraw = static_cast<int>(
        round((m_ParticleNum * m_LastList[drawIndex]->getWeight()) + 0.5));
    for (int i = 0; i < numToDraw; i++)
    {
      *m_CurrentList[targetIndex++] = *m_LastList[drawIndex];
      // don't draw too much
      if (targetIndex >= m_ParticleNum)
      {
        break;
      }
    }
    drawIndex++;
  } while (numToDraw > 0 && targetIndex < m_ParticleNum);

  // fill the rest of the particle list
  for (int i = targetIndex; i < m_ParticleNum; i++)
  {
    float particlePos = random01();
    float weightSum = 0.0;
    drawIndex = 0;
    weightSum += m_LastList[drawIndex]->getWeight();
    while (weightSum < particlePos)
    {
      weightSum += m_LastList[++drawIndex]->getWeight();
    }
    *m_CurrentList[i] = *m_LastList[drawIndex];
  }
}

template <class ParticleType>
int ParticleFilter<ParticleType>::getEffectiveParticleNum() const
{
  // does not work with normalized particle weights
  // does not work with our weights at all (algorithm of Grisetti)
  float squareSum = 0;
  for (int i = 0; i < m_ParticleNum; i++)
  {
    float weight = m_CurrentList[i]->getWeight();
    squareSum += weight * weight;
  }
  return static_cast<int>(1.0f / squareSum);
}

template <class ParticleType>
int ParticleFilter<ParticleType>::getEffectiveParticleNum2() const
{
  // does not work with normalized particle weights
  int effectiveParticleNum = 0;
  for (int i = 0; i < m_ParticleNum; i++)
  {
    if (m_CurrentList[i]->getWeight() > MIN_EFFECTIVE_PARTICLE_WEIGHT)
    {
      effectiveParticleNum++;
    }
  }
  return effectiveParticleNum;
}

template <class ParticleType>
ParticleType* ParticleFilter<ParticleType>::getBestParticle() const
{
  return m_CurrentList[0];
}

#endif
