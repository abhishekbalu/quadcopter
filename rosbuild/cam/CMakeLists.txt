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

#-----QT
find_package(Qt4 REQUIRED)
# enable/disable some Qt features
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

#ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

set(qt_srcs src/qtfile.cpp)
set(qt_hdrs src/qtfile.h)
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

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
#----Build the P3P Library
rosbuild_add_library(p3p src/p3p.cpp)
#----Build the usb_cam driver
rosbuild_add_library(usb_cam src/usb_cam.cpp)
#----Build the Blob Detection library with the functions
rosbuild_add_library(blob_detection src/blob_detection.cpp)
#rosbuild_add_library(blob_detection_simple src/blob_detection_simple.cpp)
#----Build the Marker Detection library with the functions
rosbuild_add_library(marker_detection src/marker_detection.cpp)
#----Link libraries 
target_link_libraries(marker_detection p3p yaml-cpp)
target_link_libraries(usb_cam avcodec swscale)


#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#-------------------------------------------------EXECUTABLES------------------------------------------------------
#rosbuild_add_executable(main_node src/main_node.cpp)
#target_link_libraries(main_node usb_cam marker_detection blob_detection yaml-cpp)
#rosbuild_add_executable(marker_debug src/marker_detection_debug.cpp)
#target_link_libraries(marker_debug marker_detection blob_detection yaml-cpp)
rosbuild_add_executable(dummy src/dummy.cpp)
rosbuild_add_executable(node src/node.cpp)
target_link_libraries(node marker_detection blob_detection yaml-cpp)
#----Non-QT
rosbuild_add_executable(blob_debug src/blob_detection_debug.cpp)
target_link_libraries(blob_debug blob_detection yaml-cpp)
#----QT
rosbuild_add_executable(laser_corrections src/laser_corrections.cpp)
rosbuild_add_executable(blob_debug_qt src/blob_detection_debug_qt.cpp ${uis_h} ${qt_srcs} ${qt_moc_srcs})
target_link_libraries(blob_debug_qt blob_detection yaml-cpp ${QT_LIBRARIES})
