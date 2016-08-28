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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

if($ENV{ROS_DISTRO} MATCHES "indigo")
	find_package(cmake_modules REQUIRED)
endif($ENV{ROS_DISTRO} MATCHES "indigo")
if($ENV{ROS_DISTRO} MATCHES "jade")
        find_package(cmake_modules REQUIRED)
endif($ENV{ROS_DISTRO} MATCHES "jade")

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(main_control src/main_controller_2.cpp)
#rosbuild_add_executable(main_control_test src/main_controller_test.cpp)
#rosbuild_add_executable(main_control_teleop src/main_controller_teleop.cpp)
rosbuild_add_executable(get_angles src/get_angles.cpp)
rosbuild_add_executable(asctec2px4_control src/asctec2px4_control.cpp)
#target_link_libraries(example ${PROJECT_NAME})
