#include <homer_mapping/ParticleFilter/SlamFilter.h>
#include <omp.h>

// minimum move for translation in m
const float MIN_MOVE_DISTANCE2 = 0.05 * 0.05;
// minimum turn in radiants
const float MIN_TURN_DISTANCE2 = 0.1 * 0.01;

const float M_2PI = 2 * M_PI;

SlamFilter::SlamFilter(int particleNum)
  : ParticleFilter<SlamParticle>(particleNum)
{
  m_OccupancyMap = new OccupancyMap();
  // generate initial particles
  for (int i = 0; i < m_ParticleNum; i++)
  {
    m_CurrentList[i] = new SlamParticle();
    m_LastList[i] = new SlamParticle();
  }

  float rotationErrorRotating = 0.0;
  ros::param::get("/particlefilter/error_values/rotation_error_rotating",
                  rotationErrorRotating);
  float rotationErrorTranslating = 0.0;
  ros::param::get("/particlefilter/error_values/rotation_error_translating",
                  rotationErrorTranslating);
  float translationErrorTranslating = 0.0;
  ros::param::get("/particlefilter/error_values/translation_error_translating",
                  translationErrorTranslating);
  float translationErrorRotating = 0.0;
  ros::param::get("/particlefilter/error_values/translation_error_translating",
                  translationErrorRotating);
  float moveJitterWhileTurning = 0.0;
  ros::param::get("/particlefilter/error_values/move_jitter_while_turning",
                  moveJitterWhileTurning);

  setRotationErrorRotating(rotationErrorRotating);
  setRotationErrorTranslating(rotationErrorTranslating);
  setTranslationErrorTranslating(translationErrorTranslating);
  setTranslationErrorRotating(translationErrorRotating);
  setMoveJitterWhileTurning(moveJitterWhileTurning);

  m_FirstRun = true;
  m_DoMapping = true;

  m_EffectiveParticleNum = m_ParticleNum;
  m_LastMoveTime = ros::Time::now();
}

SlamFilter::SlamFilter(SlamFilter& slamFilter)
  : ParticleFilter<SlamParticle>(slamFilter.m_ParticleNum)
{
  m_OccupancyMap = new OccupancyMap(*(slamFilter.m_OccupancyMap));
  // generate initial particles
  for (int i = 0; i < m_ParticleNum; i++)
  {
    if (slamFilter.m_CurrentList[i] == 0)
    {
      m_CurrentList[i] = 0;
    }
    else
    {
      m_CurrentList[i] = new SlamParticle(*(slamFilter.m_CurrentList[i]));
    }
    if (slamFilter.m_LastList[i] == 0)
    {
      m_LastList[i] = 0;
    }
    else
    {
      m_LastList[i] = new SlamParticle(*(slamFilter.m_LastList[i]));
    }
  }

  float rotationErrorRotating = 0.0;
  ros::param::get("/particlefilter/error_values/rotation_error_rotating",
                  rotationErrorRotating);
  float rotationErrorTranslating = 0.0;
  ros::param::get("/particlefilter/error_values/rotation_error_translating",
                  rotationErrorTranslating);
  float translationErrorTranslating = 0.0;
  ros::param::get("/particlefilter/error_values/translation_error_translating",
                  translationErrorTranslating);
  float translationErrorRotating = 0.0;
  ros::param::get("/particlefilter/error_values/translation_error_translating",
                  translationErrorRotating);
  float moveJitterWhileTurning = 0.0;
  ros::param::get("/particlefilter/error_values/move_jitter_while_turning",
                  moveJitterWhileTurning);

  setRotationErrorRotating(rotationErrorRotating);
  setRotationErrorTranslating(rotationErrorTranslating);
  setTranslationErrorTranslating(translationErrorTranslating);
  setTranslationErrorRotating(translationErrorRotating);
  setMoveJitterWhileTurning(moveJitterWhileTurning);

  m_FirstRun = slamFilter.m_FirstRun;
  m_DoMapping = slamFilter.m_DoMapping;

  m_EffectiveParticleNum = slamFilter.m_EffectiveParticleNum;
}

SlamFilter::~SlamFilter()
{
  if (m_OccupancyMap)
  {
    delete m_OccupancyMap;
  }
  for (int i = 0; i < m_ParticleNum; i++)
  {
    if (m_CurrentList[i])
    {
      delete m_CurrentList[i];
      m_CurrentList[i] = 0;
    }
    if (m_LastList[i])
    {
      delete m_LastList[i];
      m_LastList[i] = 0;
    }
  }
}

void SlamFilter::setRotationErrorRotating(float percent)
{
  m_Alpha1 = percent / 100.0;
}

void SlamFilter::resetHigh()
{
  m_OccupancyMap->resetHighSensitive();
}

void SlamFilter::setRotationErrorTranslating(float degreePerMeter)
{
  m_Alpha2 = degreePerMeter / 180.0 * M_PI;
}

void SlamFilter::setTranslationErrorTranslating(float percent)
{
  m_Alpha3 = percent / 100.0;
}

void SlamFilter::setTranslationErrorRotating(float mPerDegree)
{
  m_Alpha4 = mPerDegree / 180.0 * M_PI;
}

void SlamFilter::setMoveJitterWhileTurning(float mPerDegree)
{
  m_Alpha5 = mPerDegree / 180.0 * M_PI;
}

void SlamFilter::setMapping(bool doMapping)
{
  m_DoMapping = doMapping;
}

void SlamFilter::setOccupancyMap(OccupancyMap* occupancyMap)
{
  // delete old
  if (m_OccupancyMap)
  {
    delete m_OccupancyMap;
  }
  // copy
  m_OccupancyMap = occupancyMap;
}

vector<Pose>* SlamFilter::getParticlePoses() const
{
  vector<Pose>* particlePoses = new vector<Pose>();
  for (int i = 0; i < m_ParticleNum; i++)
  {
    particlePoses->push_back(m_CurrentList[i]->getRobotPose());
  }
  return particlePoses;
}

vector<SlamParticle*>* SlamFilter::getParticles() const
{
  vector<SlamParticle*>* particles = new vector<SlamParticle*>();
  for (int i = 0; i < m_ParticleNum; i++)
  {
    SlamParticle* particle = m_CurrentList[i];
    particles->push_back(particle);
  }
  return particles;
}

void SlamFilter::setRobotPose(Pose pose, double scatterVarXY,
                              double scatterVarTheta)
{
  // set first particle to exact position
  m_CurrentList[0]->setRobotPose(pose.x(), pose.y(), pose.theta());
  m_LastList[0]->setRobotPose(pose.x(), pose.y(), pose.theta());
  // scatter remaining particles
  for (int i = 1; i < m_ParticleNum; ++i)
  {
    const double scatterX = randomGauss() * scatterVarXY;
    const double scatterY = randomGauss() * scatterVarXY;
    const double scatterTheta = randomGauss() * scatterVarTheta;

    m_CurrentList[i]->setRobotPose(pose.x() + scatterX, pose.y() + scatterY,
                                   pose.theta() + scatterTheta);
    m_LastList[i]->setRobotPose(pose.x() + scatterX, pose.y() + scatterY,
                                pose.theta() + scatterTheta);
  }
}

vector<float> SlamFilter::getParticleWeights() const
{
  vector<float> particleWeights(m_ParticleNum);
  for (int i = 0; i < m_ParticleNum; i++)
  {
    particleWeights[i] = m_CurrentList[i]->getWeight();
  }
  return particleWeights;
}

double SlamFilter::randomGauss(float variance) const
{
  if (variance < 0)
  {
    variance = -variance;
  }
  double x1, x2, w, y1;
  do
  {
    x1 = 2.0 * random01() - 1.0;
    x2 = 2.0 * random01() - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w >= 1.0);

  w = sqrt((-2.0 * log(w)) / w);
  y1 = x1 * w;
  // now y1 is uniformly distributed
  return sqrt(variance) * y1;
}

vector<float> SlamFilter::filterOutliers(sensor_msgs::LaserScanConstPtr rawData,
                                         float maxDiff)
{
  if (rawData->ranges.size() < 2)
  {
    return rawData->ranges;
  }
  vector<float> filteredData = rawData->ranges;
  for (unsigned int i = 1; i < filteredData.size() - 1; i++)
  {
    if (abs((float)(rawData->ranges[i - 1] - rawData->ranges[i] * 2 +
                    rawData->ranges[i + 1])) > maxDiff * 2)
    {
      filteredData[i] = 0;
    }
  }
  if (fabs(rawData->ranges[0] - rawData->ranges[1]) > maxDiff)
  {
    filteredData[0] = 0;
  }
  if (fabs(rawData->ranges[rawData->ranges.size() - 1] -
           rawData->ranges[rawData->ranges.size() - 2]) > maxDiff)
  {
    filteredData[rawData->ranges.size() - 1] = 0;
  }

  return filteredData;
}

void SlamFilter::filter(Transformation2D trans,
                        sensor_msgs::LaserScanConstPtr laserData)
{
  sensor_msgs::LaserScanPtr laserScan =
      boost::make_shared<sensor_msgs::LaserScan>(
          *laserData);  // copy const ptr to be able to change values
  laserScan->ranges = filterOutliers(laserData, 0.3);

  if (m_FirstRun)
  {
    ROS_INFO_STREAM("first run!");
    m_FirstRun = false;
    // only do mapping, save first pose as reference
    if (m_DoMapping)
    {
      tf::Transform transform(tf::createQuaternionFromYaw(0.0),
                              tf::Vector3(0.0, 0.0, 0.0));
      m_OccupancyMap->insertLaserData(laserScan, transform);
    }

    resample();
    measure(laserScan);
    normalize();
    sort(0, m_ParticleNum - 1);

    m_LastMoveTime = ros::Time::now();
    return;
  }

  bool moving = sqr(trans.x()) + sqr(trans.y()) > 0.0000001 ||
                sqr(trans.theta()) > 0.000001;

  if (moving)
  {
    m_LastMoveTime = ros::Time::now();
  }

  bool particlesScattered = m_EffectiveParticleNum < m_ParticleNum / 10;

  bool justMoved = (ros::Time::now() - m_LastMoveTime).toSec() < 0.5;

  if (justMoved || particlesScattered)
  {
    resample();
    drift(trans);
    measure(laserScan);
    normalize();
    sort(0, m_ParticleNum - 1);
  }

  if (m_DoMapping && justMoved)
  {
    Pose likeliestPose = getLikeliestPose();
    tf::Transform transform(
        tf::createQuaternionFromYaw(likeliestPose.theta()),
        tf::Vector3(likeliestPose.x(), likeliestPose.y(), 0.0));
    m_OccupancyMap->insertLaserData(laserScan, transform);
  }
}

/**
 *  For the probabilistic motion model of the robot we use the following three
 * parameters:
 *  - When the robot starts, the initial orientation may have errors (a few
 * degrees). (m_InitialOrientationError)
 *  - The distance of the robot movement may be wrong (a percentage of the moved
 * distance). (m_TranslationError)
 *  - The orientation of the robot when the motion was finished may be wrong (a
 * percentage of the rotation) (m_RotationError).
 *  [cf. "An Efficient FastSLAM Algorithm for Generating Maps of Large-Scale
 * Cyclic Environments
 *   from Raw Laser Range Measurements", Dirk Haenelt et. al.]
 *  We use Gaussian-Distributions to estimate the error.
 *  The expected value of the errors are zero.
 */

void SlamFilter::drift(Transformation2D odoTrans)
{
  SlamParticle* particle = m_CurrentList[0];
  // get stored pose
  Pose pose = particle->getRobotPose();

  float deltaX =
      odoTrans.x() * cos(pose.theta()) - odoTrans.y() * sin(pose.theta());
  float deltaY =
      odoTrans.x() * sin(pose.theta()) + odoTrans.y() * cos(pose.theta());
  float deltaS = std::fabs(deltaX) + std::fabs(deltaY);

  // always leave one particle with pure displacement
  // move pose
  float posX = pose.x() + deltaX;
  float posY = pose.y() + deltaY;
  float theta = pose.theta() + odoTrans.theta();
  // save new pose
  particle->setRobotPose(posX, posY, theta);

  for (int i = 1; i < m_ParticleNum; i++)
  {
    SlamParticle* particle = m_CurrentList[i];
    // get stored pose
    Pose pose = particle->getRobotPose();

    float posX =
        pose.x() + deltaX + randomGauss(m_Alpha3 * std::fabs(deltaX) +
                                        m_Alpha5 * std::fabs(odoTrans.theta()));
    float posY =
        pose.y() + deltaY + randomGauss(m_Alpha3 * std::fabs(deltaY) +
                                        m_Alpha5 * std::fabs(odoTrans.theta()));
    float theta = pose.theta() + odoTrans.theta() +
                  randomGauss(m_Alpha1 * std::fabs(odoTrans.theta()) +
                              m_Alpha2 * std::fabs(deltaS));

    // save new pose
    while (theta > M_PI)
      theta -= M_2PI;
    while (theta <= -M_PI)
      theta += M_2PI;

    particle->setRobotPose(posX, posY, theta);
  }
}

void SlamFilter::measure(sensor_msgs::LaserScanPtr laserData)
{
  if (m_OccupancyMap)
  {
    std::vector<MeasurePoint> measurePoints =
        m_OccupancyMap->getMeasurePoints(laserData);

    for (int i = 0; i < m_ParticleNum; i++)
    {
      SlamParticle* particle = m_CurrentList[i];
      if (particle)
      {
        particle->setWeight(m_OccupancyMap->computeScore(
            particle->getRobotPose(), measurePoints));
      }
    }
  }
  m_EffectiveParticleNum = getEffectiveParticleNum2();
}

void SlamFilter::reduceParticleNumber(int newParticleNum)
{
  if (newParticleNum < m_ParticleNum)
  {
    SlamParticle** newCurrentList = new SlamParticle*[newParticleNum];
    SlamParticle** newLastList = new SlamParticle*[newParticleNum];

    for (int i = 0; i < newParticleNum; i++)
    {
      newCurrentList[i] = m_CurrentList[i];
      newLastList[i] = m_LastList[i];
    }

    for (int i = newParticleNum + 1; i < m_ParticleNum; i++)
    {
      delete m_CurrentList[i];
      delete m_LastList[i];
    }
    delete[] m_CurrentList;
    delete[] m_LastList;

    m_CurrentList = newCurrentList;
    m_LastList = newLastList;

    m_ParticleNum = newParticleNum;
    normalize();
  }
}

Pose SlamFilter::getLikeliestPose()
{
  float percentage = 2;
  float sumX = 0, sumY = 0, sumDirX = 0, sumDirY = 0;
  int numParticles = static_cast<int>(percentage / 100.0 * m_ParticleNum);
  if (0 == numParticles)
  {
    numParticles = 1;
  }
  for (int i = 0; i < numParticles; i++)
  {
    Pose pose = m_CurrentList[i]->getRobotPose();
    sumX += pose.x();
    sumY += pose.y();
    // calculate sum of vectors in unit circle
    sumDirX += cos(pose.theta());
    sumDirY += sin(pose.theta());
  }
  float meanTheta = atan2(sumDirY, sumDirX);
  float meanX = sumX / numParticles;
  float meanY = sumY / numParticles;
  return Pose(meanX, meanY, meanTheta);
}

OccupancyMap* SlamFilter::getLikeliestMap() const
{
  return m_OccupancyMap;
}

void SlamFilter::getPoseVariances(int particleNum, float& poseVarianceX,
                                  float& poseVarianceY, float& poseVarianceT)
{
  // the particles of m_CurrentList are sorted by their weights
  if (particleNum > m_ParticleNum || particleNum <= 0)
  {
    particleNum = m_ParticleNum;
  }
  // calculate average pose
  float averagePoseX = 0;
  float averagePoseY = 0;
  float averagePoseT = 0;
  float robotX = 0.0;
  float robotY = 0.0;
  float robotT = 0.0;
  for (int i = 0; i < particleNum; i++)
  {
    m_CurrentList[i]->getRobotPose(robotX, robotY, robotT);
    averagePoseX += robotX;
    averagePoseY += robotY;
    averagePoseT += robotT;
  }
  averagePoseX /= (float)particleNum;
  averagePoseY /= (float)particleNum;
  averagePoseT /= (float)particleNum;

  // calculate standard deviation of pose
  poseVarianceX = 0.0;
  poseVarianceY = 0.0;
  poseVarianceT = 0.0;
  for (int i = 0; i < particleNum; i++)
  {
    m_CurrentList[i]->getRobotPose(robotX, robotY, robotT);
    poseVarianceX += (averagePoseX - robotX) * (averagePoseX - robotX);
    poseVarianceY += (averagePoseY - robotY) * (averagePoseY - robotY);
    poseVarianceT += (averagePoseT - robotT) * (averagePoseT - robotT);
  }
  poseVarianceX /= (float)particleNum;
  poseVarianceY /= (float)particleNum;
  poseVarianceT /= (float)particleNum;
}

double SlamFilter::evaluateByContrast()
{
  return m_OccupancyMap->evaluateByContrast();
}

void SlamFilter::applyMasking(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  m_OccupancyMap->applyMasking(msg);
}
