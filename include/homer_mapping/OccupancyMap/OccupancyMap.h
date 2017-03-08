#ifndef OCCUPANCYMAP_H
#define OCCUPANCYMAP_H

#include <iostream>
#include <list>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <homer_nav_libs/Math/Box2D.h>
#include <homer_nav_libs/Math/Point2D.h>
#include <homer_nav_libs/Math/Pose.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

class QImage;

using namespace std;

/**
 * Structure to store the start and end point of each laser range in the current
 * scan
 * @param sensorPos position of the laser in the current scan (in base_link
 * frame)
 * @param endPos position of the end point of the laser frame in the current
 * scan (in base_link frame)
 * @param free indicates if the laser range hit an obstacle (false) or not
 * (true)
 */
struct RangeMeasurement {
    geometry_msgs::Point sensorPos;
    geometry_msgs::Point endPos;
    float range;
    bool free;
};

/**
 * Used in struct MeasurePoint to specify if a measurement point is at the
 * border of a scan segment
 */
enum BorderType { NoBorder, LeftBorder, RightBorder };

/**
 * Structure to store a measurement point for computeLaserScanProbability()
 * @param hitPos Position of measured obstacle (robot coordinates)
 * @param frontPos Position to check for NOT_KNOWN terrain
 *                   This is needed to assure that front- and backside of
 * obstacles can be distinguished
 * @param border specifies if the measurement point is at the border of a scan
 * segment
 */
struct MeasurePoint {
    Point2D hitPos;
    Point2D frontPos;
    BorderType borderType;
};

/**
 * @class OccupancyMap
 *
 * @author Malte Knauf, Stephan Wirth, Susanne Maur (RX), David Gossow (RX),
 * Susanne Thierfelder (R16)
 *
 * @brief This class holds and manages an occupancy map.
 *
 * An occupancy map is a map where free space and occupied space are marked.
 * This map stores values
 * for free and occupied space in an (2D-)unsigned char array. This array can be
 * seen as a graylevel image.
 * The darker a cell, the higher the probability that this cell is occupied by
 * an obstacle.
 * The size of the map and the size of one cell can be defined in the setup file
 * with the values
 * MAP_SIZE and MAP_CELL_SIZE. The origin of the coordinate system of the map is
 * the center of the array.
 * The x-axis is heading front, the y-axis points to the left (like the robot's
 * coordinate system).
 * The mapping data has to be inserted via the method insertLaserData().
 */
class OccupancyMap {
   public:
    static const int8_t INACCESSIBLE = 100;
    static const int8_t OBSTACLE = 99;
    static const int8_t OCCUPIED = 98;
    static const int8_t UNKNOWN = 50;
    static const int8_t NOT_SEEN_YET = -1;
    static const int8_t FREE = 0;

    /**
     * The default constructor calls initMembers().
     */
    OccupancyMap();

    /**
     * Constructor for a loaded map.
     */
    OccupancyMap(float*& occupancyProbability, geometry_msgs::Pose origin,
                 float resolution, int width, int height,
                 Box2D<int> exploredRegion);

    /**
     * Copy constructor, copies all members inclusive the arrays that lie behind
     * the pointers.
     * @param occupancyMap Source for copying
     */
    OccupancyMap(const OccupancyMap& occupancyMap);

    /**
     * Method to init all members with default values from the configuration
     * file. All arrays are initialized.
     */
    void initMembers();

    /**
     * Assignment operator, copies all members (deep copy!)
     * @param source Source to copy from
     * @return Reference to copied OccupancyMap
     */
    OccupancyMap& operator=(const OccupancyMap& source);

    /**
     * Deletes all dynamically allocated memory.
     */
    ~OccupancyMap();

    /*
    /**
     * @return The resolution of the map in m.
     */
    //    int resolution() const;

    geometry_msgs::Pose origin() const;

    /**
     * @return Width of the map.
     */
    int width() const;

    /**
     * @return Height of the map.
     */
    int height() const;

    /**
     * This method is used to reset all HighSensitive areas
     */
    void resetHighSensitive();

    /**
     * @return Probability of pixel p being occupied.
     */
    float getOccupancyProbability(Eigen::Vector2i p);

    /**
     * @brief This function inserts the data of a laserscan into the map.
     *
     * With the given data, start and end cells of a laser beam are computed and
     * given to the
     * method markLineFree().
     * If the measurement is smaller than VALID_MAX_RANGE, markOccupied() is
     * called for the endpoint.
     * @param laserData The laser data msg.
     */
    void insertLaserData(sensor_msgs::LaserScan::ConstPtr laserData,
                         tf::Transform transform);

    void insertRanges(vector<RangeMeasurement> ranges,
                      ros::Time stamp = ros::Time::now());

    /**
     * @brief gives a list specially processed coordinates to be used for
     * computeLaserScanProbability
     */
    std::vector<MeasurePoint> getMeasurePoints(
        sensor_msgs::LaserScanConstPtr laserData);

    /**
     * This method computes a score that describes how good the given hypothesis
     * matches with the map
     * @param robotPose The pose of the robot
     * @return The "fitting factor". The higher the factor, the better the
     * fitting.
     *         This factor is NOT normalized, it is a positive float between 0
     * and FLOAT_MAX
     */
    float computeScore(Pose robotPose, std::vector<MeasurePoint> measurePoints);

    /**
     * @return QImage of size m_metaData.width, m_metaData.height with values of
     * m_OccupancyProbability scaled to 0-254
     */
    QImage getProbabilityQImage(int trancparencyThreshold,
                                bool showInaccessible) const;

    // puma2::ColorImageRGB8* getUpdateImage( bool withMap=true ); TODO

    /**
     * Returns an "image" of the obstacles e.g. seen in the 3D scans
     * @returns image with dark red dots in areas where the obstacles were seen
     */
    // puma2::ColorImageRGB8* getObstacleImage ( ); TODO

    /**
     * Returns an "image" of occupancy probability image.
     * @param[out] data vector containing occupancy probabilities. 0 = free, 100
     * = occupied, -1 = NOT_KNOWN
     * @param[out] width Width of data array
     * @param[out] height Height of data array
     * @param[out] resolution Resolution of the map (m_metaData.resolution)
     */
    void getOccupancyProbabilityImage(vector<int8_t>& data,
                                      nav_msgs::MapMetaData& metaData);

    /**
     * This method marks free the position of the robot (according to its
     * dimensions).
     */
    void markRobotPositionFree();

    /**
     * @brief Computes the contrast of a single pixel.
     * @param prob probability value (100=occupied, 50=NOT_KNOWN, 0=free) of a
     * pixel.
     * @return Contrast value from 0 (no contrast) to 1 (max. contrast) of this
     * pixel
     */
    double contrastFromProbability(int8_t prob);

    /**
     * @brief This method computes the sharpness of the occupancy grid
     * @return Contrast value from 0 (no contrast) to 1 (max. contrast) of the
     * map
     */
    double evaluateByContrast();

    /// GETTERS

    Box2D<int> getExploredRegion() { return m_ExploredRegion; }
    Box2D<int> getChangedRegion() { return m_ChangedRegion; }

    /**
     * Sets cells of this map to free or occupied according to maskMap
     */
    void applyMasking(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void changeMapSize(int x_add_left, int y_add_up, int x_add_right,
                       int y_add_down);

   protected:
    /**
     * This method increments m_MeasurementCount for pixel p.
     * @param p Pixel that has been measured.
     */
    void incrementMeasurementCount(Eigen::Vector2i p);

    /**
     * This method increments the occupancy count int m_OccupancyCount for pixel
     * p.
     * @param p Occupied pixel.
     */
    void incrementOccupancyCount(Eigen::Vector2i p);

    /**
     * This method increments m_MeasurementCount and if neccessary
     * m_OccupancyCount for all pixels.
     */
    void applyChanges();

    void clearChanges();

    /**
     * This method scales the counts of all pixels down to the given value.
     * @param maxCount Maximum value to which all counts are set.
     */
    void scaleDownCounts(int maxCount);

    /**
      * This function paints a line from a start pixel to an end pixel.
      * The computation is made with the Bresenham algorithm.
      * @param data array on which the line shall be painted
      * @param startPixel starting coordinates of the beam
      * @param endPixel ending coordinates of the beam
      * @param value The value with which the lines are marked.
      */
    template <class DataT>
    void drawLine(DataT* data, Eigen::Vector2i& startPixel,
                  Eigen::Vector2i& endPixel, char value);

    /**
     * This method computes the values for m_OccupancyProbabilities from
     * m_MeasurementCount and m_OccupancyCount.
     */
    void computeOccupancyProbabilities();

    /**
     * This method sets all values of m_CurrentChanges to NO_CHANGE.
     */
    void clearCurrentChanges();

    /**
     * This method resets all values of m_MinChangeX, m_MaxChangeX, m_MinChangeY
     * and m_MaxChangeY.
     * This means that no current changes are assumed.
     */
    void resetChangedRegion();

    /**
     * This method updates the values of m_MinChangeX, m_MaxChangeX,
     * m_MinChangeY and m_MaxChangeY to current changes.
     * The area around the current robot pose will be included to the changed
     * region.
     * @param robotPose The current pose of the robot.
     */
    void updateChangedRegion(Pose robotPose);

    /**
     * This method sets all values of m_MinChangeX, m_MaxChangeX, m_MinChangeY
     * and m_MaxChangeY
      * to initial values so that the complete map will be processed.
     */
    void maximizeChangedRegion();

    /**
     * This method resets all values of m_ExploredX, m_MaxExploredX,
     * m_MinExploredY and m_MaxExploredY.
     */
    void resetExploredRegion();

    /**
     * Deletes all allocated members.
     */
    void cleanUp();

    /**
     * Stores the metadata of the map
     */
    nav_msgs::MapMetaData m_metaData;

    /**
     * Stores the size of the map arrays, i.e. m_metaData.width *
     * m_metaData.height
     * for faster computation.
     */
    unsigned m_ByteSize;

    /**
     * Array to store occupancy probability values.
     * Values between 0 and 1.
     */
    float* m_OccupancyProbability;

    // Counts how often a pixel is hit by a measurement.
    unsigned short* m_MeasurementCount;

    // Counts how often a pixel is hit by a measurement that says the pixel is
    // occupied.
    unsigned short* m_OccupancyCount;

    // Used for setting flags for cells, that have been modified during the
    // current update.
    unsigned char* m_CurrentChanges;

    // Used for high Sensitive areas
    unsigned short* m_HighSensitive;

    /**
     * Store values from config files.
     */
    // minimum range classified as free in case of errorneous laser measurement
    float m_FreeReadingDistance;
    // enables checking to avoid matching front- and backside of an obstacle,
    // e.g. wall
    bool m_BacksideChecking;
    // leaves a small border around obstacles unchanged when inserting a laser
    // scan
    bool m_ObstacleBorders;
    // minimum distance in m between two samples for probability calculation
    float m_MeasureSamplingStep;

    // bool to reset high_sensitive Areas on the next iteration
    bool m_reset_high;

    /**
     * Defines a bounding box around the changes in the current map.
     */
    Box2D<int> m_ChangedRegion;

    /**
     * Defines a bounding box around the area in the map, which is already
     * explored.
     */
    Box2D<int> m_ExploredRegion;

    /**
     * ros transform listener
     */
    tf::TransformListener m_tfListener;

    /**
     * ros transformation laser to base_link
     */
    tf::StampedTransform m_laserTransform;
    tf::Transform m_latestMapTransform;
};
#endif
