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

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})



find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#rosbuild_add_executable(fix_interaction src/fix_interaction.cpp src/udp_server.cpp src/udp_client.cpp src/input_lecture.cpp)
#rosbuild_add_executable(fix_receiver src/fix_receiver.cpp src/udp_server.cpp src/udp_client.cpp src/input_lecture.cpp)
rosbuild_add_executable(mobile_interaction_XBox_version src/mobile_interaction_XBox_version.cpp src/udp_server.cpp src/udp_client.cpp src/input_lecture.cpp)
rosbuild_add_executable(mobile_interaction src/mobile_interaction.cpp src/udp_server.cpp src/udp_client.cpp src/input_lecture.cpp)
#rosbuild_add_executable(mobile_stream src/mobile_stream.cpp src/udp_server.cpp src/udp_client.cpp src/input_lecture.cpp)
rosbuild_add_executable(mobile_controller src/mobile_controller.cpp src/input_lecture.cpp) 
#rosbuild_add_executable(TOL src/mobile_TOL.cpp)
#rosbuild_add_executable(controller src/HX_controller_pub.cpp)
#rosbuild_add_executable(listener src/HX_controller_sub.cpp)
#target_link_libraries(controller SDL)

