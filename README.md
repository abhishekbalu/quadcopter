![](http://i.imgur.com/SgiJzWe.png)
Blob detection test
![](http://www.ultraimg.com/images/2016/07/10/xy_filter_results_2.png)
OpticalFlow filter test
![](http://i.imgur.com/yambfyT.png)
X backward and forwards movement by a frame registered with the algorithm
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
Simply follow the instructions on the following site (all rights belong to the author):
http://milq.github.io/install-opencv-ubuntu-debian/

# 3 - PX4 Toolchain
You'll need to also install the PX4 Toolchain to use some of the packages. Make sure you are in the home directory:
```sh
$ sudo usermod -a -G dialout $USER
$ sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
$ sudo apt-get update
$ sudo apt-get install python-argparse git-core wget zip \
    python-empy qtcreator cmake build-essential genromfs -y
$ sudo add-apt-repository ppa:openjdk-r/ppa
$ sudo apt-get update
$ sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y
$ sudo apt-get remove modemmanager
$ sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded -y
$ sudo apt-get update
$ sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo build-essential \
    libftdi-dev libtool zlib1g-dev \
    python-empy gcc-arm-none-eabi -y
```
If the resulting gcc-arm-none-eabi version produces build errors for PX4/Firmware master, please use <http://dev.px4.io/starting-installing-linux-boutique.html> (refer to the TOOLCHAIN section to install gcc-arm-none-eabi).
Now you'll need to build the code:
```sh
$ mkdir -p ~/src
$ cd ~/src
$ git clone https://github.com/PX4/Firmware.git
$ cd Firmware
$ git submodule update --init --recursive
$ make px4fmu-v2_default
```
## 4 - Gazeebo and Hector
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
Add the following lines to your bashrc and place the quad_urf models in your home directory:

source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH="~/quad_urdf/models" 

After that you'll need to:
```sh
$ mkdir ~/.gazebo/models
$ cd ~/.gazebo/models
$ wget --mirror -p --convert-links -P . http://models.gazebosim.org
$ mv models.gazebosim.org/* .
$ rm -rf models.gazebosim.org
```

## 5 - .bashrc
Your .bashrc may have the following aspect at the end (add the aliases). In these aliases we use rqt_image_view to quickly see an image. However you should have rqt running from the beggining and you can use the gui to display images side-by-side (like shown at the beggining of this document: <http://wiki.ros.org/rqt_image_view>. Rqt_image_view does not need OpenCV support. This bashrc is mainly local to your machine for testing purposes.
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
alias view_images='rosrun rqt_image_view rqt_image_view'
alias view_detections='rostopic echo cam/detections'
alias calibrate='rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/usb_cam/image_raw camera:=/usb_cam'
```
The bashrc file on the quad is a different as it has suitable alias for flying not for the simulation or testing:
```sh
source /opt/ros/indigo/setup.bash
export ROS_PACKAGE_PATH=~/quadcopter:$ROS_PACKAGE_PATH

alias mavros_launch='roslaunch mavros px4.launch'
alias takeoff='rostopic pub /syscommand std_msgs/String "takeoff"'
alias land='rostopic pub /syscommand std_msgs/String "land"'
alias publish_setpoints='rostopic pub /mavros/setpoint_position/local -r 10 geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x:0.0, y: 0.0, z:0.0}, orientation: {w: 1.0}}}''
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
To use it either put it in catkin/src (catkin version) or put it in ros folder (rosbuild version)
## 8 - NVidia drivers (optional)
Try to do this. It didn't work for me, might work for you:
```
sudo apt-get install nvidia-bumblebee bumblebee
```

## 8 - Theory
The folder called theory has some svg files and images that explain most of the modules (even some with UML) used in this project in a clear manner.

## 9 - P3P Library
All rights to the p3p library included belong to Laurent Kneip, ETH Zurich and the logic behind the algorithm can be read at: <http://publications.asl.ethz.ch/files/kneip11novel.pdf>
## 10 - How to arm the quad and fly manual (RC1)
  - Turn the Nvidia on and connect the PX4. Log in via ssh. 
  - Turn the battery on and make sure the motor checkup happens (two small beeps and the rotors turn). The PX4 light should be fading in and out and blue
  - Launch mavros. 
  - Turn the RC transmitter (Spektrum) on. Make sure you can see the connection on the mavros. You'll probably see something like MANUAL CONTROL MODE. The PX4 light should flash green when you do this.
  - Arm the quad by pushing the blinking red LED button for 2 seconds, make sure the left and right levers are in the down center position. The PX4 light should flash white when you do this.
  - To begin flying move the left lever to the down right position. After arming the RC controller the motors should start on iddle.
![](http://www.ultraimg.com/images/2016/07/10/spektrumleverpositions.png)
Disarming:
   - Move the RC Transmitter left lever to the down-left position
  - Disconnect the battery. The PX4 light should be fading in and out and blue
  - Turn off the RC Transmitter.

## 11 - How to go into OFFBOARD mode
After you've done the first 5 steps in the previous section, on the command line if you are experiencing problems you may need to publish some setpoints before going into OFFBOARD mode. If you don't do this you might get OFFBOARD MODE REJECTED errors by the FCU, however it will probably work ok.
```
rostopic pub /mavros/setpoint_position/local -r 10 geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x:0.0, y: 0.0, z:0.0}, orientation: {w: 1.0}}}' 
```
After this, you'll need to cheange the rightmost switch on the top of the Spektrum from 0 (HOLD) to 1 to change the RC Channel and signal the PX4 you want to go into OFFBOARD mode. You may see an error from the FCU in your mavros launch window about failsafe mode being on or off.

To check if you were succesful:
```
rostopic echo /mavros/state 
```
And see if the armed parameter is True (should be if you pressed the red LED button for 2s) and if the mode is OFFBOARD. If you don't want to use the RC Transmitter you may also force the change (this was not tested):
```
rosrun mavros mavsys mode -c OFFBOARD
```
## 12 - Data
All data, equations and flow charts of the ideas and experiments conducted can be found [here](https://www.dropbox.com/sh/mmmgt4p6f89z934/AABFnw_2Zww_G8jRH_ggLmYFa?dl=0).

## 13 - Advice
1 - When using the optical flow some of the baseline readings may not be 0 if there are reflections or LEDs flashing. Be careful of this as it will cause velocities to have high values and it will cause instability for the estimators and the x and y values will diverge to infinity.

2 - Be careful with the referentials don't get confused. Optical Flow baseline is the height of the quad, 30 cm.

3 - Since you login to the NVidia via ssh consider using some sort of graphical program to allow you visualize data onboard such as X. (ssh x for Unix, something like XLaunch with X11 setup in Putty). Also consider using some sort of terminal emulator like [tmux](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-tmux-on-ubuntu-12-10--2) or [terminator](http://technicalworldforyou.blogspot.pt/2012/11/install-terminator-terminal-emulator-in.html).

4 - If you want to change the frequency of the optical flow data that is sent via mavros you have to edit a file in the PX4 SD Card, in etc/extras.txt. If this doesn't exist create it. Then simply add the following to it (to set a rate of 200Hz for example):
```
mavlink stream -d /dev/tty/ACM0 -s OPTICAL_FLOW_RAD -r 200
```
However, believing what is said [here](http://wiki.ros.org/px4flow_node) the optical flow data from acessing directly the USB is published at 250 Hz with no need to configure.

5 - To connect via Serial to the PX4 (taken from the PX4 Official website and documentation)
| Pixhawk 1/2 |       | FTDI |                  |
|-------------|-------|------|------------------|
| 1           | 5V    |      | n/c              |
| 2           | S4 TX |      | n/c              |
| 3           | S4 RX |      | n/c              |
| 4           | S5 TX | 5    | FTDI RX (yellow) |
| 5           | S5 RX | 4    | FTDI TX (orange) |
| 6           | GND   | 1    | FTDI GND (black) |
Bellow is the connector pinout and a picture of the Pixhawk connected via Serial:
![](http://i.imgur.com/Un7pBfy.png)
![](http://i.imgur.com/NoTyiGF.jpg)
You may want to use an FTDI Cable for this purpose or use a protoboard with an FTDI Chip and some headers like so:
![](http://i.imgur.com/gpMCH4E.jpg)
6 - If the NVidia and PX4 are both connected via battery follow the steps:
  - Find the ip of the quad **sudo arp-scan --interface=wlan0 --localnet** (on our local network QuadBase5 the ip is normally 11.0.3.1 or 11.0.1.5. If you are on windows, ping these two, and you'll probably find it)
  - Connect the battery
  - Turn off USB connections
  - Turn on USB connections
  - To fly, reboot the PX4 by pressing the small button on the side with a pencil or with the toothpick that is under one of the ESCs
  - Turn the RC on
  - Arm the quad (press the LED)
  - Run your launches
7 - How to fix a broken propeller
8 - How to charge the batteries
# Steps for charging batteries in a safe manor
Connections:
 - Turn on the Charging station by flipping the switch on the power supply
 - Connect the red T plug to the charging station
 - Connect the white voltage balancer to the charging station

Settings:
 - Select the type of battery, change type with the Batt. Type button and confirm with Enter
 - Confirm the Charging Station is on Charge mode, by using Dec and Inc buttons to select it
 - Press Enter to select the charging amperage and Enter again to confirm
 - Select the end voltage and confirm it with Enter

Charging:
 - Press and Hold the Enter Button
 - Check if the selected number of battery’s cell is equal to the number of cell read by the station
 - Press Enter to start the charging process
 - Each press of the Dec. button cycles the status of the charging screen
 - Pressing the Inc. change the screen to display the actual voltage of each battery's cell

