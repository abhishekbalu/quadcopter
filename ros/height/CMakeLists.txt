cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#----Include C++11 syntax flags
SET(CMAKE_CXX_FLAGS "-std=c++11")
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#-------------------------------------------------MESSAGES AND SERVICES------------------------------------------------------
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#-------------------------------------------------LIBRARIES------------------------------------------------------
#common commands for building c++ executables and libraries
rosbuild_add_library(pid src/pid.cpp)
rosbuild_add_library(Kalman_1D src/Kalman_1D.cpp)
rosbuild_add_library(Kalman_2D src/Kalman_2D.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#-------------------------------------------------EXECUTABLES------------------------------------------------------
#Laser
#rosbuild_add_executable(laser_transform src/laser_transform.cpp)
rosbuild_add_executable(laser_filter src/laser_filter.cpp)
#Individual Filters
rosbuild_add_executable(z_pose src/z_pose.cpp)
rosbuild_add_executable(xy_pose src/xy_pose.cpp)
rosbuild_add_executable(simOF src/simulated_OptFlow.cpp)
#Controller
rosbuild_add_executable(controller src/controller.cpp)
#global filter
rosbuild_add_executable(filter src/filter.cpp) 

target_link_libraries(filter Kalman_1D Kalman_2D)
target_link_libraries(controller pid)
target_link_libraries(z_pose Kalman_1D)
target_link_libraries(xy_pose Kalman_2D)
