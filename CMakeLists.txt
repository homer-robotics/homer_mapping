cmake_minimum_required(VERSION 2.8.3)
project(homer_mapping)

find_package(
	catkin REQUIRED COMPONENTS
		roscpp
		homer_mapnav_msgs
		sensor_msgs
		nav_msgs
		homer_nav_libs
		tf
		roslib
)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(tf REQUIRED)

set(CMAKE_BUILD_TYPE Release)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
		roscpp
		homer_mapnav_msgs
		homer_nav_libs
		nav_msgs
		tf
		roslib
		LIBRARIES homerOccupancyMap homerParticleFilter
)

add_library(homerOccupancyMap
	src/OccupancyMap/OccupancyMap.cpp
	
)

target_link_libraries(
	homerOccupancyMap
		${catkin_LIBRARIES}
		${tf_LIBRARIES}
)
set(
	ParticleFilter_SRC
		src/ParticleFilter/HyperSlamFilter.cpp
		src/ParticleFilter/SlamParticle.cpp
		src/ParticleFilter/SlamFilter.cpp
		src/ParticleFilter/Particle.cpp
)

add_library(
	homerParticleFilter
		${ParticleFilter_SRC}
)

target_link_libraries(
	homerParticleFilter
		${catkin_LIBRARIES}
)

add_executable(homer_mapping src/slam_node.cpp)

target_link_libraries(
	homer_mapping
		${catkin_LIBRARIES}
		homerParticleFilter
		homerOccupancyMap
)

add_dependencies(
	homer_mapping
		${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS homerOccupancyMap homerParticleFilter homer_mapping
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
