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

if($ENV{ROS_DISTRO} MATCHES "indigo")
	find_package(cmake_modules REQUIRED)
endif($ENV{ROS_DISTRO} MATCHES "indigo")
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/rab3D.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(rab3D src/rab3D.cpp)
#rosbuild_add_executable(rab3D_emit src/RaB3D.cpp src/estimator_emit.cpp src/rab3D_emit_sim.cpp)
#rosbuild_add_executable(rab3D_robot src/RaB3D.cpp src/estimator_robot.cpp src/egomotion_2.cpp src/rab3D_robot_sim.cpp)
rosbuild_add_executable(rab3D_robot src/egomotion_2.cpp src/rab3D_robot_sim.cpp)
target_link_libraries(rab3D_robot librt.so)
