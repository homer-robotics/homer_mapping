#include <homer_mapping/ParticleFilter/Particle.h>

Particle::Particle(float weight, int id) {
  m_Weight = weight;
  m_Id = id;
}

Particle::~Particle() {
}

