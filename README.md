# homer_mapping

## Introduction 

The homer_mapping package consists of a node with the same name. This node is responsible for the localization and mapping of the robot using the odometry of the robot and a laser scanner. The SLAM problem is solved by the particle filter algorithm. The node continuously expects odometry values and laser data and sends corrected pose estimates about the topic /pose and tf transformation /map -> /base_link at constant intervals. In addition, the robot can be located on a previously loaded map and a currently created map can be saved. There is the option to switch the mapping on or off. When loading a map, the mapping is automatically switched off.

## Topics 


#### Publisher 
* `/pose (geometry_msgs/PoseStamped)`: The currently determined pose relative to the map (in the frame /map) of the robot from the particle filter.
* `/homer_mapping/slam_map (nav_msgs/OccupancyGrid)`: The current map of the robot.



#### Subscriber

* `/odom (nav_msgs/Odometry)`: Current odometry values. Those are needed by the particle filter.
  `/scan (sensor_msgs/LaserScan)`: The current laser range measurements.
* `/homer_mapping/userdef_pose (geometry_msgs/Pose)`: With this topic the current pose calculated by the particle filter can be set to a user defined pose. The particle filter now continues to work with this one..
* `/homer_mapping/do_mapping (map_messages/DoMapping)`: With this set to true the mapping will be activated. `false` will deactivate.
* `/map_manager/reset_maps (std_msgs/Empty)`: Here you can reset the current map.
* `/map_manager/loaded_map (nav_msgs/OccupancyGrid)`: With this topic the current map can be replaced by another (loaded) map.
* `/map_manager/mask_slam (nav_msgs/OccupancyGrid)`: The OccupancyGrid of this topic contains information which parts of the current map should be replaced by other values (free or occupied).

Translated with www.DeepL.com/Translator

## Launch Files 

* `homer_mapping.launch`: This launchfile loads the parameter file `homer_mapping.yaml` and starts the node homer_mapping and the node map_manager in the package of the same name.
* `homer_mapping_rviz.launch`: This launchfile has the same functionality as the one above, but starts rviz in addition.

## Parameter 


### homer_mapping.yaml



* `/homer_mapping/size:` Size describes the size of one side of the map in meters. The map is square
* `/homer_mapping/resolution:` Resolution is the length of one (square) cell of the map in meters
* `/homer_mapping/backside_checking:` If set to true, it prevents the front and back of a thicker wall from being matched to it.
* `/homer_mapping/obstacle_borders:` If set to "true", a small border of unknown area is left around entered obstacles.
* `/homer_mapping/measure_sampling_step:` Minimum distance in meters that must be present between two consecutive measuring points in the laser scan in order to use it for the pose calculation.
* `/homer_mapping/laser_scanner/free_reading_distance:` Minimum distance in meters assumed to be unobstructed if the current measuring point is faulty
* `/particlefilter/error_values/rotation_error_rotating:` Rotation error in percent that occurs when rotating the robot
* `/particlefilter/error_values/rotation_error_translating:` Rotation error in degrees that occurs when driving one meter
* `/particlefilter/error_values/translation_error_translating:` Percentage translation error that occurs when driving straight ahead
* `/particlefilter/error_values/translation_error_rotating:` Translation error in meters that occurs when rotating by one degree
* `/particlefilter/error_values/move_jitter_while_turning:` Scatter of the newly calculated pose in meters per degree of rotation
* `/particlefilter/hyper_slamfilter/particlefilter_num:` Number of particle filters in the hyperparticle filter (set to 1 by default)
* `/particlefilter/particle_num:` Number of particles in each particle filter
* `/particlefilter/max_rotation_per_second:` Maximum rotation in radians per second that the robot is allowed to rotate without suspending the mapping.
* `/particlefilter/wait_time:` Minimum time that must have elapsed between two mapping steps
* `/particlefilter/update_min_move_angle:` Minimum rotation in degrees that must be performed to perform a particle filter step...
* `/particlefilter/update_min_move_dist:` ...or minimum distance in meters that the robot moves straight ahead...
* `/particlefilter/max_update_interval:` ...or minimum waiting time in which the robot stands still.
* `/selflocalization/scatter_var_xy:` Scattering of particles in x/y direction in meters when setting a user-defined pose
* `/selflocalization/scatter_var_theta:` Scatter of particle alignment in Radiant when setting a custom pose

Translated with www.DeepL.com/Translator


