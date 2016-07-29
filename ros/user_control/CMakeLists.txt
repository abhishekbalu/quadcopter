cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#added to use normal_distribution class - may do bad things
#set(CMAKE_CXX_FLAGS "-std=c++11")

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(user_terminal src/user_terminal.cpp)

#rosbuild_add_executable(command_line src/command_line.cpp)

#rosbuild_add_executable(multi_command_line src/multi_command_line.cpp)
rosbuild_add_executable(user_control_sim src/user_control_sim.cpp)
rosbuild_add_executable(user_control_quad src/user_control_quad.cpp src/udp_server.cpp src/udp_client.cpp)

rosbuild_add_executable(user_control_pc src/user_control_pc.cpp src/udp_client.cpp src/udp_server)
#target_link_libraries(example ${PROJECT_NAME})
