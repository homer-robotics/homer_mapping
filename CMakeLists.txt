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
		homer_tools
)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
find_package(tf REQUIRED)

set(CMAKE_BUILD_TYPE Release)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${QT_INCLUDE_DIR}
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
		roscpp
		homer_mapnav_msgs
		homer_nav_libs
)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

add_library( OccupancyMap
	src/OccupancyMap/OccupancyMap.cpp
	
)

target_link_libraries(
	OccupancyMap
		${catkin_LIBRARIES}
		${QT_LIBRARIES}
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
	ParticleFilter
		${ParticleFilter_SRC}
)

target_link_libraries(
	ParticleFilter
		${catkin_LIBRARIES}
)

add_executable(homer_mapping src/slam_node.cpp)

target_link_libraries(
	homer_mapping
		${catkin_LIBRARIES}
		ParticleFilter
		OccupancyMap
)

add_dependencies(
	homer_mapping
		${catkin_EXPORTED_TARGETS}
)
