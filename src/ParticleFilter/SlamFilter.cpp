#include <homer_mapping/ParticleFilter/SlamFilter.h>
#include <omp.h>

// minimum move for translation in m
const float MIN_MOVE_DISTANCE2 = 0.001 * 0.001;
// minimum turn in radiants
const float MIN_TURN_DISTANCE2 = 0.01 * 0.01;

const float M_2PI = 2 * M_PI;

SlamFilter::SlamFilter(int particleNum)
    : ParticleFilter<SlamParticle>(particleNum) {
    m_OccupancyMap = new OccupancyMap();
    // generate initial particles
    for (int i = 0; i < m_ParticleNum; i++) {
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
    ros::param::get(
        "/particlefilter/error_values/translation_error_translating",
        translationErrorTranslating);
    float translationErrorRotating = 0.0;
    ros::param::get(
        "/particlefilter/error_values/translation_error_translating",
        translationErrorRotating);
    float moveJitterWhileTurning = 0.0;
    ros::param::get("/particlefilter/error_values/move_jitter_while_turning",
                    moveJitterWhileTurning);
    ros::param::get("/particlefilter/max_rotation_per_second",
                    m_MaxRotationPerSecond);

    int updateMinMoveAngleDegrees;
    ros::param::get("/particlefilter/update_min_move_angle",
                    updateMinMoveAngleDegrees);
    m_UpdateMinMoveAngle = Math::deg2Rad(updateMinMoveAngleDegrees);
    ros::param::get("/particlefilter/update_min_move_dist",
                    m_UpdateMinMoveDistance);
    double maxUpdateInterval;
    ros::param::get("/particlefilter/max_update_interval", maxUpdateInterval);
    m_MaxUpdateInterval = ros::Duration(maxUpdateInterval);

    setRotationErrorRotating(rotationErrorRotating);
    setRotationErrorTranslating(rotationErrorTranslating);
    setTranslationErrorTranslating(translationErrorTranslating);
    setTranslationErrorRotating(translationErrorRotating);
    setMoveJitterWhileTurning(moveJitterWhileTurning);

    m_FirstRun = true;
    m_DoMapping = true;

    m_EffectiveParticleNum = m_ParticleNum;
    m_LastUpdateTime = ros::Time(0);
    m_LastMoveTime = ros::Time::now();
}

SlamFilter::SlamFilter(SlamFilter& slamFilter)
    : ParticleFilter<SlamParticle>(slamFilter.m_ParticleNum) {
    m_OccupancyMap = new OccupancyMap(*(slamFilter.m_OccupancyMap));
    // generate initial particles
    for (int i = 0; i < m_ParticleNum; i++) {
        if (slamFilter.m_CurrentList[i] == 0) {
            m_CurrentList[i] = 0;
        } else {
            m_CurrentList[i] = new SlamParticle(*(slamFilter.m_CurrentList[i]));
        }
        if (slamFilter.m_LastList[i] == 0) {
            m_LastList[i] = 0;
        } else {
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
    ros::param::get(
        "/particlefilter/error_values/translation_error_translating",
        translationErrorTranslating);
    float translationErrorRotating = 0.0;
    ros::param::get(
        "/particlefilter/error_values/translation_error_translating",
        translationErrorRotating);
    float moveJitterWhileTurning = 0.0;
    ros::param::get("/particlefilter/error_values/move_jitter_while_turning",
                    moveJitterWhileTurning);
    ros::param::get("/particlefilter/max_rotation_per_second",
                    m_MaxRotationPerSecond);

    int updateMinMoveAngleDegrees;
    ros::param::get("/particlefilter/update_min_move_angle",
                    updateMinMoveAngleDegrees);
    m_UpdateMinMoveAngle = Math::deg2Rad(updateMinMoveAngleDegrees);
    ros::param::get("/particlefilter/update_min_move_dist",
                    m_UpdateMinMoveDistance);
    double maxUpdateInterval;
    ros::param::get("/particlefilter/max_update_interval", maxUpdateInterval);
    m_MaxUpdateInterval = ros::Duration(maxUpdateInterval);

    setRotationErrorRotating(rotationErrorRotating);
    setRotationErrorTranslating(rotationErrorTranslating);
    setTranslationErrorTranslating(translationErrorTranslating);
    setTranslationErrorRotating(translationErrorRotating);
    setMoveJitterWhileTurning(moveJitterWhileTurning);

    m_FirstRun = slamFilter.m_FirstRun;
    m_DoMapping = slamFilter.m_DoMapping;

    m_EffectiveParticleNum = slamFilter.m_EffectiveParticleNum;

    m_LastUpdateTime = slamFilter.m_LastUpdateTime;

    m_ReferencePoseOdometry = slamFilter.m_ReferencePoseOdometry;
    m_ReferenceMeasurementTime = slamFilter.m_ReferenceMeasurementTime;
}

SlamFilter::~SlamFilter() {
    if (m_OccupancyMap) {
        delete m_OccupancyMap;
    }
    for (int i = 0; i < m_ParticleNum; i++) {
        if (m_CurrentList[i]) {
            delete m_CurrentList[i];
            m_CurrentList[i] = 0;
        }
        if (m_LastList[i]) {
            delete m_LastList[i];
            m_LastList[i] = 0;
        }
    }
}

void SlamFilter::setRotationErrorRotating(float percent) {
    m_Alpha1 = percent / 100.0;
}

void SlamFilter::resetHigh() { m_OccupancyMap->resetHighSensitive(); }

void SlamFilter::setRotationErrorTranslating(float degreePerMeter) {
    m_Alpha2 = degreePerMeter / 180.0 * M_PI;
}

void SlamFilter::setTranslationErrorTranslating(float percent) {
    m_Alpha3 = percent / 100.0;
}

void SlamFilter::setTranslationErrorRotating(float mPerDegree) {
    m_Alpha4 = mPerDegree / 180.0 * M_PI;
}

void SlamFilter::setMoveJitterWhileTurning(float mPerDegree) {
    m_Alpha5 = mPerDegree / 180.0 * M_PI;
}

void SlamFilter::setScanMatchingClusterSize(float minClusterSize) {
    minClusterSize = minClusterSize;
}

void SlamFilter::setMapping(bool doMapping) { m_DoMapping = doMapping; }

void SlamFilter::setOccupancyMap(OccupancyMap* occupancyMap) {
    // delete old
    if (m_OccupancyMap) {
        delete m_OccupancyMap;
    }
    // copy
    m_OccupancyMap = occupancyMap;
}

vector<Pose>* SlamFilter::getParticlePoses() const {
    vector<Pose>* particlePoses = new vector<Pose>();
    for (int i = 0; i < m_ParticleNum; i++) {
        float robotX, robotY, robotTheta;
        SlamParticle* particle = m_CurrentList[i];
        particle->getRobotPose(robotX, robotY, robotTheta);
        particlePoses->push_back(Pose(robotX, robotY, robotTheta));
    }
    return particlePoses;
}

vector<SlamParticle*>* SlamFilter::getParticles() const {
    vector<SlamParticle*>* particles = new vector<SlamParticle*>();
    for (int i = 0; i < m_ParticleNum; i++) {
        SlamParticle* particle = m_CurrentList[i];
        particles->push_back(particle);
    }
    return particles;
}

void SlamFilter::setRobotPose(Pose pose, double scatterVarXY,
                              double scatterVarTheta) {
    // set first particle to exact position
    m_CurrentList[0]->setRobotPose(pose.x(), pose.y(), pose.theta());
    m_LastList[0]->setRobotPose(pose.x(), pose.y(), pose.theta());
    // scatter remaining particles
    for (int i = 1; i < m_ParticleNum; ++i) {
        const double scatterX = randomGauss() * scatterVarXY;
        const double scatterY = randomGauss() * scatterVarXY;
        const double scatterTheta = randomGauss() * scatterVarTheta;

        m_CurrentList[i]->setRobotPose(pose.x() + scatterX, pose.y() + scatterY,
                                       pose.theta() + scatterTheta);
        m_LastList[i]->setRobotPose(pose.x() + scatterX, pose.y() + scatterY,
                                    pose.theta() + scatterTheta);
    }
}

vector<float> SlamFilter::getParticleWeights() const {
    vector<float> particleWeights(m_ParticleNum);
    for (int i = 0; i < m_ParticleNum; i++) {
        particleWeights[i] = m_CurrentList[i]->getWeight();
    }
    return particleWeights;
}

double SlamFilter::randomGauss(float variance) const {
    if (variance < 0) {
        variance = -variance;
    }
    double x1, x2, w, y1;
    do {
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
                                         float maxDiff) {
    if (rawData->ranges.size() < 2) {
        return rawData->ranges;
    }
    vector<float> filteredData = rawData->ranges;
    for (unsigned int i = 1; i < filteredData.size() - 1; i++) {
        if (abs((float)(rawData->ranges[i - 1] - rawData->ranges[i] * 2 +
                        rawData->ranges[i + 1])) > maxDiff * 2) {
            filteredData[i] = 0;
        }
    }
    if (fabs(rawData->ranges[0] - rawData->ranges[1]) > maxDiff) {
        filteredData[0] = 0;
    }
    if (fabs(rawData->ranges[rawData->ranges.size() - 1] -
             rawData->ranges[rawData->ranges.size() - 2]) > maxDiff) {
        filteredData[rawData->ranges.size() - 1] = 0;
    }

    return filteredData;
}

void SlamFilter::filter(Pose currentPose,
                        sensor_msgs::LaserScanConstPtr laserData,
                        ros::Time measurementTime,
                        ros::Duration& FilterDuration) {
    // if first run, initialize data
    if (m_FirstRun) {
        m_FirstRun = false;
        // only do mapping, save first pose as reference
        if (m_DoMapping) {
            tf::Transform transform(tf::createQuaternionFromYaw(0.0),
                                    tf::Vector3(0.0, 0.0, 0.0));
            m_OccupancyMap->insertLaserData(laserData, transform);
        }
        m_CurrentLaserData = boost::make_shared<sensor_msgs::LaserScan>(
            *laserData);  // copy const ptr to be able to change values; //test
        m_ReferencePoseOdometry = currentPose;
        m_ReferenceMeasurementTime = measurementTime;

        measure();
        ROS_INFO_STREAM("first run!");
        normalize();
        sort(0, m_ParticleNum - 1);
        return;
    }
    // m_CurrentLaserConfig = laserConf;
    m_CurrentPoseOdometry = currentPose;
    m_CurrentLaserData = boost::make_shared<sensor_msgs::LaserScan>(
        *laserData);  // copy const ptr to be able to change values
    m_CurrentLaserData->ranges = filterOutliers(laserData, 0.3);

    Transformation2D trans = m_CurrentPoseOdometry - m_ReferencePoseOdometry;

    bool moving = sqr(trans.x()) + sqr(trans.y()) > 0 || sqr(trans.theta()) > 0;

    // do not resample if move to small and last move is min 0.5 sec away
    // if (sqr(trans.x()) + sqr(trans.y()) < MIN_MOVE_DISTANCE2 &&
    // sqr(trans.theta()) < MIN_TURN_DISTANCE2 &&
    //(ros::Time::now() - m_LastMoveTime).toSec() > 1.0)
    if (!moving && (ros::Time::now() - m_LastMoveTime).toSec() > 1.0)
    // if(false)
    {
        ROS_DEBUG_STREAM("Move too small, will not resample.");
        if (m_EffectiveParticleNum < m_ParticleNum / 10) {
            resample();
            ROS_INFO_STREAM("Particles too scattered, resampling.");
            // filter steps
            drift();
            measure();
            normalize();

            sort(0, m_ParticleNum - 1);
        }
    } else {
        if (moving) {
            m_LastMoveTime = ros::Time::now();
        }
        resample();
        // filter steps
        drift();
        measure();
        normalize();

        sort(0, m_ParticleNum - 1);
    }

    Pose likeliestPose = getLikeliestPose(measurementTime);  // test
    Transformation2D transSinceLastUpdate = likeliestPose - m_LastUpdatePose;

    ostringstream stream;
    stream.precision(2);
    stream << "Transformation since last update: angle="
           << Math::rad2Deg(transSinceLastUpdate.theta())
           << " dist=" << transSinceLastUpdate.magnitude() << "m" << endl;

    bool update =
        ((std::fabs(transSinceLastUpdate.theta()) > m_UpdateMinMoveAngle) ||
         (transSinceLastUpdate.magnitude() > m_UpdateMinMoveDistance)) &&
        ((measurementTime - m_LastUpdateTime) > m_MaxUpdateInterval);

    if (m_DoMapping && update) {
        stream << "Updating map.";
        double elapsedSeconds =
            (measurementTime - m_ReferenceMeasurementTime).toSec();
        double thetaPerSecond;
        if (elapsedSeconds == 0.0) {
            thetaPerSecond = trans.theta();
        } else {
            thetaPerSecond = trans.theta() / elapsedSeconds;
        }
        float poseVarianceX, poseVarianceY;
        getPoseVariances(50, poseVarianceX, poseVarianceY);

        if (std::fabs(thetaPerSecond) < m_MaxRotationPerSecond &&
            poseVarianceX < 0.05 && poseVarianceY < 0.05) {
            updateMap();
            m_LastUpdatePose = likeliestPose;
            m_LastUpdateTime = measurementTime;
        } else {
            ROS_WARN_STREAM("No mapping performed - variance to high");
        }
    } else {
        stream << "No map update performed.";
    }
    ROS_DEBUG_STREAM(stream.str());
    // safe last used pose and laserdata as reference

    m_ReferencePoseOdometry = m_CurrentPoseOdometry;
    m_ReferenceMeasurementTime = measurementTime;
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

void SlamFilter::drift() {
    float rx = m_ReferencePoseOdometry.x();
    float ry = m_ReferencePoseOdometry.y();
    float rt = m_ReferencePoseOdometry.theta();
    float cx = m_CurrentPoseOdometry.x();
    float cy = m_CurrentPoseOdometry.y();
    float ct = m_CurrentPoseOdometry.theta();

    Transformation2D odoTrans = m_CurrentPoseOdometry - m_ReferencePoseOdometry;

    // find out if driving forward or backward
    bool backwardMove = false;
    float scalar = odoTrans.x() * cosf(rt) + odoTrans.y() * sinf(rt);
    if (scalar <= 0) {
        backwardMove = true;
    }
    float distance = sqrt(sqr(odoTrans.x()) + sqr(odoTrans.y()));
    float deltaRot1, deltaTrans, deltaRot2;
    if (distance < sqrt(MIN_MOVE_DISTANCE2)) {
        deltaRot1 = odoTrans.theta();
        deltaTrans = 0.0;
        deltaRot2 = 0.0;
    } else if (backwardMove) {
        deltaRot1 = atan2(ry - cy, rx - cx) - rt;
        deltaTrans = -distance;
        deltaRot2 = ct - rt - deltaRot1;
    } else {
        deltaRot1 = atan2(odoTrans.y(), odoTrans.x()) - rt;
        deltaTrans = distance;
        deltaRot2 = ct - rt - deltaRot1;
    }

    while (deltaRot1 >= M_PI) deltaRot1 -= M_2PI;
    while (deltaRot1 < -M_PI) deltaRot1 += M_2PI;
    while (deltaRot2 >= M_PI) deltaRot2 -= M_2PI;
    while (deltaRot2 < -M_PI) deltaRot2 += M_2PI;

    // always leave one particle with pure displacement
    SlamParticle* particle = m_CurrentList[0];
    // get stored pose
    float robotX, robotY, robotTheta;
    particle->getRobotPose(robotX, robotY, robotTheta);
    Pose pose(robotX, robotY, robotTheta);
    // move pose
    float posX = pose.x() + deltaTrans * cos(pose.theta() + deltaRot1);
    float posY = pose.y() + deltaTrans * sin(pose.theta() + deltaRot1);
    float theta = pose.theta() + deltaRot1 + deltaRot2;
    while (theta > M_PI) theta -= M_2PI;
    while (theta <= -M_PI) theta += M_2PI;
    // save new pose
    particle->setRobotPose(posX, posY, theta);
    int i = 1;

    // calculating parallel on 8 threats
    // TODO: ERROR ON RESET MAPS
    //  omp_set_num_threads(4);
    //  #pragma omp parallel for
    for (i = 1; i < m_ParticleNum; i++) {
        SlamParticle* particle = m_CurrentList[i];
        // get stored pose
        float robotX, robotY, robotTheta;
        particle->getRobotPose(robotX, robotY, robotTheta);
        Pose pose(robotX, robotY, robotTheta);
        // move pose
        float estDeltaRot1 =
            deltaRot1 -
            randomGauss(m_Alpha1 * fabs(deltaRot1) + m_Alpha2 * deltaTrans);
        float estDeltaTrans =
            deltaTrans -
            randomGauss(m_Alpha3 * deltaTrans +
                        m_Alpha4 * (fabs(deltaRot1) + fabs(deltaRot2)));
        float estDeltaRot2 =
            deltaRot2 -
            randomGauss(m_Alpha1 * fabs(deltaRot2) + m_Alpha2 * deltaTrans);

        float posX = pose.x() +
                     estDeltaTrans * cos(pose.theta() + estDeltaRot1) +
                     randomGauss(m_Alpha5 * fabs(estDeltaRot1 + estDeltaRot2));
        float posY = pose.y() +
                     estDeltaTrans * sin(pose.theta() + estDeltaRot1) +
                     randomGauss(m_Alpha5 * fabs(estDeltaRot1 + estDeltaRot2));
        float theta = pose.theta() + estDeltaRot1 + estDeltaRot2;

        // save new pose
        while (theta > M_PI) theta -= M_2PI;
        while (theta <= -M_PI) theta += M_2PI;

        particle->setRobotPose(posX, posY, theta);
    }
}

void SlamFilter::measure() {
    if (m_OccupancyMap) {
        m_MeasurePoints = m_OccupancyMap->getMeasurePoints(m_CurrentLaserData);

        // calculating parallel on 8 threats
        omp_set_num_threads(8);
        int i = 0;
        //#pragma omp parallel for
        for (i = 0; i < m_ParticleNum; i++) {
            SlamParticle* particle = m_CurrentList[i];
            if (!particle) {
                ROS_ERROR_STREAM("ERROR: Particle is NULL-pointer!");
            } else {
                // calculate weights
                float robotX, robotY, robotTheta;
                particle->getRobotPose(robotX, robotY, robotTheta);
                Pose pose(robotX, robotY, robotTheta);
                float weight =
                    m_OccupancyMap->computeScore(pose, m_MeasurePoints);
                particle->setWeight(weight);
            }
        }
    }
    m_EffectiveParticleNum = getEffectiveParticleNum2();
}

void SlamFilter::updateMap() {
    m_OccupancyMap->insertLaserData(m_CurrentLaserData, m_latestTransform);
}

void SlamFilter::printParticles() const {
    cout << endl << "### PARTICLE LIST ###" << endl;
    cout << right << fixed;
    cout.width(5);
    for (int i = 0; i < m_ParticleNum; i++) {
        SlamParticle* p_particle = m_CurrentList[i];
        if (p_particle) {
            float robotX, robotY, robotTheta;
            p_particle->getRobotPose(robotX, robotY, robotTheta);
            cout << "Particle " << i << ": (" << robotX << "," << robotY << ","
                 << robotTheta * 180.0 / M_PI << "), weight:\t"
                 << p_particle->getWeight() << endl;
        }
    }
    cout << "### END OF LIST ###" << endl;
}

void SlamFilter::reduceParticleNumber(int newParticleNum) {
    if (newParticleNum < m_ParticleNum) {
        SlamParticle** newCurrentList = new SlamParticle*[newParticleNum];
        SlamParticle** newLastList = new SlamParticle*[newParticleNum];

        for (int i = 0; i < newParticleNum; i++) {
            newCurrentList[i] = m_CurrentList[i];
            newLastList[i] = m_LastList[i];
        }

        for (int i = newParticleNum + 1; i < m_ParticleNum; i++) {
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

Pose SlamFilter::getLikeliestPose(ros::Time poseTime) {
    float percentage = 0.4;  // TODO param? //test
    float sumX = 0, sumY = 0, sumDirX = 0, sumDirY = 0;
    int numParticles = static_cast<int>(percentage / 100 * m_ParticleNum);
    if (0 == numParticles) {
        numParticles = 1;
    }
    for (int i = 0; i < numParticles; i++) {
        float robotX, robotY, robotTheta;
        m_CurrentList[i]->getRobotPose(robotX, robotY, robotTheta);
        sumX += robotX;
        sumY += robotY;
        // calculate sum of vectors in unit circle
        sumDirX += cos(robotTheta);
        sumDirY += sin(robotTheta);
    }
    float meanTheta = atan2(sumDirY, sumDirX);
    float meanX = sumX / numParticles;
    float meanY = sumY / numParticles;
    // broadcast transform map -> base_link
    tf::Transform transform(
        tf::createQuaternionFromYaw(meanTheta),
        tf::Vector3(sumX / numParticles, sumY / numParticles, 0.0));
    tf::TransformBroadcaster tfBroadcaster;
    m_latestTransform = transform;

    tfBroadcaster.sendTransform(tf::StampedTransform(
        m_latestTransform, poseTime, "/map", "/base_link"));
    return Pose(meanX, meanY, meanTheta);
}

OccupancyMap* SlamFilter::getLikeliestMap() const { return m_OccupancyMap; }

void SlamFilter::getPoseVariances(int particleNum, float& poseVarianceX,
                                  float& poseVarianceY) {
    // the particles of m_CurrentList are sorted by their weights
    if (particleNum > m_ParticleNum || particleNum <= 0) {
        particleNum = m_ParticleNum;
    }
    // calculate average pose
    float averagePoseX = 0;
    float averagePoseY = 0;
    float robotX = 0.0;
    float robotY = 0.0;
    float robotTheta = 0.0;
    for (int i = 0; i < particleNum; i++) {
        m_CurrentList[i]->getRobotPose(robotX, robotY, robotTheta);
        averagePoseX += robotX;
        averagePoseY += robotY;
    }
    averagePoseX /= particleNum;
    averagePoseY /= particleNum;

    // calculate standard deviation of pose
    poseVarianceX = 0.0;
    poseVarianceY = 0.0;
    for (int i = 0; i < particleNum; i++) {
        m_CurrentList[i]->getRobotPose(robotX, robotY, robotTheta);
        poseVarianceX += (averagePoseX - robotX) * (averagePoseX - robotX);
        poseVarianceY += (averagePoseY - robotY) * (averagePoseY - robotY);
    }
    poseVarianceX /= particleNum;
    poseVarianceY /= particleNum;
}

double SlamFilter::evaluateByContrast() {
    return m_OccupancyMap->evaluateByContrast();
}

void SlamFilter::applyMasking(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    m_OccupancyMap->applyMasking(msg);
}
