cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_CXX_FLAGS "-std=c++11")
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_library(pid src/pid.cpp)
rosbuild_add_library(Kalman_1D src/Kalman_1D.cpp)
rosbuild_add_library(Kalman_2D src/Kalman_2D.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

# Comment this to use the Mavros Optical Flow
#rosbuild_add_executable(height_filter src/height_filter.cpp)

rosbuild_add_executable(xy_pose src/xy_pose.cpp)
#rosbuild_add_executable(xy_pose_original src/xy_pose_original.cpp)
#rosbuild_add_executable(xy_pose_no_mavros src/xy_pose_no_mavros.cpp)
rosbuild_add_executable(z_pose src/z_pose.cpp)
rosbuild_add_executable(odometry src/xy_odometry.cpp)
rosbuild_add_executable(controller_quad src/controller_quad.cpp)
rosbuild_add_executable(controller_quad_z src/controller_quad_z.cpp)
rosbuild_add_executable(repeater src/repeater_node.cpp)
#target_link_libraries(xy_pose_no_mavros Kalman_2D)
#target_link_libraries(xy_pose_original Kalman_2D)
target_link_libraries(xy_pose Kalman_2D)
target_link_libraries(z_pose Kalman_1D)
target_link_libraries(controller_quad pid)
target_link_libraries(controller_quad_z pid)

#target_link_libraries(example ${PROJECT_NAME})
