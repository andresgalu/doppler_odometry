cmake_minimum_required(VERSION 3.0.2)
project(doppler_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
# Set build type to Release by default
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif(NOT CMAKE_BUILD_TYPE)
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
else(CMAKE_BUILD_TYPE STREQUAL "Release")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
endif(CMAKE_BUILD_TYPE STREQUAL "Release")
message(STATUS "Flags: " ${CMAKE_CXX_FLAGS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#Looking for Eigen
find_package(Eigen3 3.3.7 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

file( GLOB LIB_HEADERS 
${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
${PROJECT_SOURCE_DIR}/include/rtl/*.hpp )

file( GLOB LIB_SOURCES
${PROJECT_SOURCE_DIR}/src/odom.cpp
${PROJECT_SOURCE_DIR}/src/doppler.cpp)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/${PROJECT_NAME}
  include/rtl
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/main.cpp ${LIB_HEADERS} ${LIB_SOURCES})

## Specify libraries to link a library or executable target against
#Use ROS
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#Use Eigen
include_directories(include ${EIGEN3_INCLUDE_DIR})

