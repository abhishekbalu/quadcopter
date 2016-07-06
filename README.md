![](http://i.imgur.com/5hADUgR.gif)
![](http://i.imgur.com/OouIExg.png)

# quadcopter
## Structure

> This project consists of the ROS files for computer-vision, simulation,
> and controlled flight of a quadcopter using the PX4 autopilot with an NVidia Jetson TK1 as the onboard computer,
> a hector Gazebo simulator and a USB Camera/Hokuyo Laser.
> The objective is to evaluate which of the two methods, the USB Camera
> or the Hokuyo Laser is the best for quadcopter pose-estimation.

This project has two folders, ros and catkin. ros is the rosbuild workspace and catkin is the catkin workspace
  - **ros** (height, height-quad, cam, px-ros-pkg-rosbuild, yaml-cpp packages, simulation -- this package is for simulator)
  - **catkin** (height, height-quad, cam, px-ros-pkg, yaml-cpp packages)

## Set up

Requirements: It is assumed you have a stable, clean instalation of Ubuntu 14.04, either on a virtual machine or on your computer.

## 1 - ROS
You will need to install ROS Indigo (latest stable version as of 06/07/2016) <http://wiki.ros.org/indigo/Installation/Ubuntu>:
```sh
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
$ sudo apt-get update
$ rosdep update
$ sudo rosdep init
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
$ source /opt/ros/indigo/setup.bash
$ sudo apt-get install python-rosinstall
```
After this is done, add source /opt/ros/indigo/setup.bash AND export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH to your .bashrc (sudo nano .bashrc in your home directory).
# 2 - OpenCV
You'll also need OpenCV which is a library for image processing (<http://opencv.org>)
Simply follow the instructions on the following site:
https://help.ubuntu.com/community/OpenCV

# 3 - PX4 Toolchain
You'll need to also install the PX4 Toolchain to use some of the packages. Make sure you are in the home directory:
```sh
$ sudo usermod -a -G dialout $USER
$ sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
$ sudo apt-get update
$ sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
$ sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y
$ sudo apt-get remove modemmanager
$ sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded -y
$ sudo apt-get update
$ sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo build-essential \
    libftdi-dev libtool zlib1g-dev \
    python-empy gcc-arm-none-eabi -y
```
If the resulting gcc-arm-none-eabi version produces build errors for PX4/Firmware master, please use <http://dev.px4.io/starting-installing-linux-boutique.html>
Now you'll need to build the code (also install git, sudo apt-get install git):
$ mkdir -p ~/src
$ cd ~/src
$ git clone https://github.com/PX4/Firmware.git
$ cd Firmware
$ git submodule update --init --recursive
$ make px4fmu-v2_default
## 4 - Gazeebo and Hactor
```sh
$ sudo apt-get install ros-indigo-mavros
$ sudo apt-get install ros-indigo-hector-quadrotor
$ sudo apt-get install ros-indigo-hector-gazebo
$ sudo apt-get install ros-indigo-hector-quadrotor-gazebo
$ sudo apt-get install ros-indigo-hector-quadrotor-controller-gazebo
$ sudo apt-get install ros-indigo-hector-quadrotor-teleop
$ sudo apt-get install ros-indigo-hector-quadrotor-description
$ sudo apt-get install ros-indigo-hector-quadrotor-gazebo-plugins
$ sudo apt-get install ros-indigo-controller-manager
$ sudo apt-get install ros-indigo-message-to-tf
$ sudo apt-get install ros-indigo-gazebo-ros-pkgs
$ sudo apt-get install ros-indigo-gazebo-ros-control
```

For the usbcam and an aditional teleop python package:
```sh
$ sudo apt-get install ros-indigo-usb-cam
$ sudo apt-get install ros-indigo-image-view
$ sudo apt-get install ros-indigo-teleop-twist-keyboard
```
Then you'll need to add the gazebo sources:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo latest.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get upgrade
```
Add the following lines to your bashrc:

source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH="/home/quadbase/ros/quads/quad_common/quad_urdf/models" 

After that you'll need to:
```sh
$ mkdir ~/.gazebo/models
$ cd ~/.gazebo/models
$ wget --mirror -p --convert-links -P . http://models.gazebosim.org
$ mv models.gazebosim.org/* .
$ rm -rf models.gazebosim.org
```

## 5 - .bashrc
Your .bashrc should have the following aspect at the end (add the aliases):
```sh
source /opt/ros/indigo/setup.bash
export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH
export GAZEBO_MODEL_PATH="/home/pedro/quad_urdf/models"
alias launch='roslaunch ~/ros/simulation/launch/simulation_quad.launch'
alias teleop='rosrun teleop_twist_keyboard teleop_twist_keyboard.py'
alias takeoff='rostopic pub /syscommand std_msgs/String "takeoff"'
alias land='rostopic pub /syscommand std_msgs/String "land"'
alias cam_launch='roslaunch ~/ros/cam/launch/usb_cam.launch'
alias cd_catkin_exec='cd catkin/devel/lib'
alias view_rgb='rosrun image_view image_view image=/cam/rgb_image'
alias view_bin='rosrun image_view image_view image=/cam/binary_image'
alias view_detections='rostopic echo cam/detections'
alias calibrate='rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/usb_cam/image_raw camera:=/usb_cam'
```


## 6 - yaml.cpp 
```sh
$ git https://github.com/jbeder/yaml-cpp
$ cd yaml-cpp/src
$ mkdir build
$ cd ..
$ cd ..
$ cmake -G "Unix Makefiles"
$ make
```

## 7 - optical flow library
The library should already be in the folders above however you can get it from here aswell:
<https://github.com/cvg/px-ros-pkg>
To use it:
put it in catkin/src (catkin version)
put it in ros
## 8 - NVidia drivers (optional)
Try to do this. It didn't work for me, might work for you:
```
sudo apt-get install nvidia-bumblebee bumblebee
```

## 8 - Theory
The folder called theory has some svg files and images that explain most of the modules (even some with UML) used in this project in a clear manner.

## 9 - P3P Library
All rights to the p3p library included belong to Laurent Kneip, ETH Zurich.
