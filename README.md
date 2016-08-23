*Blob detection with red and blue marker detection, blob info and windows for light adaptation*
![](http://i.imgur.com/bTpOeNh.png)
*Here you can see a video of the light adaption working, while the camera estimation for position doesn't change*

[![](http://i.imgur.com/8Iq8fND.png)](https://www.youtube.com/watch?v=NCk6dnQ-WS0&feature=youtu.be "light adaptation algorithm in blob detection ")

*OpticalFlow/Sonar filter test with filtering (notice that the raw OF frame and the flying frame are different)*
![](http://i.imgur.com/5OC1Vw0.png)
*X backward and forwards movement by a frame registered with the algorithm*
![](http://i.imgur.com/yambfyT.png)
*Hokuyo URG pose of a series of X and Y movements and then turning the quad 90 degrees on the XY plane - yaw*
![](http://i.imgur.com/N9kTdRk.png)
![](http://i.imgur.com/BOZ6qY1.png)
*Using a model and knowing the velocity from the OF and using a filter, the OF can also estimate X and Y position (this is sort of a bonus feature). Here is a graph of an XY movement, with both OF and laser adjusted to the same frame (flying frame)*:
![](http://i.imgur.com/qRQlvvw.png)

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

Requirements: It is assumed you have a stable, clean instalation of Ubuntu 14.04, either on a virtual machine or on your computer. You'll need a quadcopter with a Nvidia Jetson TK1 with Tegra Ubuntu installed (Linux4Tegra), you'll need a PX4 Pixhawk onboard controller, an OpticalFlow board from PX4, and either a USB Cam or Hokuyo Laser.

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


## 4.5 - QGroundControl
QGroundControl is a great tool to have on your own computer to debug many sensors and quadcopter controllers (also to communicate and gather info from the quad). To install go [here](http://qgroundcontrol.com/installing-qgroundcontrol/) and [here](https://github.com/mavlink/qgroundcontrol/releases/).
```sh
tar xvjf QGroundControl.tar.bz2
sudo apt-get install espeak libespeak-dev libudev-dev libsdl1.2-dev
```
To run QGroundControl:
```sh
cd qgroundcontrol
./qgroundcontrol-start.sh
```

## 5 - .bashrc
Your .bashrc may have the following aspect at the end (add the aliases). In these aliases we use rqt_image_view to quickly see an image. However you should have rqt running from the beggining and you can use the gui to display images side-by-side (like shown at the beggining of this document: <http://wiki.ros.org/rqt_image_view>. Rqt_image_view does not need OpenCV support. This bashrc is mainly local to your machine for testing purposes.
**LOCAL**
```sh
source /opt/ros/indigo/setup.bash
export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=~/ros:$ROS_PACKAGE_PATH
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH="~/quad_urdf/models"
alias launch='roslaunch ~/ros/simulation/launch/simulation_quad.launch'
alias teleop='rosrun teleop_twist_keyboard teleop_twist_keyboard.py'
alias takeoff='rostopic pub /syscommand std_msgs/String "takeoff"'
alias land='rostopic pub /syscommand std_msgs/String "land"'
alias cam_launch='roslaunch ~/ros/cam/launch/usb_cam.launch'
alias cd_catkin_exec='cd catkin/devel/lib'
alias view_images='rosrun rqt_image_view rqt_image_view'
alias view_detections='rostopic echo cam/detections'
alias calibrate='rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/usb_cam/image'
alias tf='cd /var/tmp && rosrun tf view_frames && evince frames.pdf'
alias qgroundcontrol='cd ~/Downloads/qgroundcontrol && ./qgroundcontrol-start.sh'
alias px4flow_launch='roslaunch px4flow_node px4flow.launch'
foo(){
   rostopic type $1 | rosmsg show
}
alias show_msgtype=foo
```
The bashrc file on the quads is a different as it has suitable alias for flying not for the simulation or testing:

**QUAD1**
```sh
source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/quadcopter:${ROS_PACKAGE_PATH}
export ROS_PACKAGE_PATH=~/ros:${ROS_PACKAGE_PATH}
alias mavros_launch='roslaunch  ~/catkin_ws/src/danielpa/launch/pixhawk.launch'
alias takeoff='rostopic pub /syscommand std_msgs/String "takeoff"'
alias land='rostopic pub /syscommand std_msgs/String "land"'
#alias publish_setpoints='rostopic pub /mavros/setpoint_position/local -r 10 geometry_msgs/PoseStamped '{header: {stamp: $
alias shutdown='sudo shutdown -h now'
alias px4flow_launch='roslaunch px4flow_node px4flow.launch'
```
**QUAD2**
```sh
source /opt/ros/indigo/setup.bash
source ~/ros/quad_control_catkin/devel/setup.bash
export ROS_PACKAGE_PATH=~/ros/quad_control_rosbuild:${ROS_PACKAGE_PATH}
export ROS_PACKAGE_PATH=~/quadcopter:${ROS_PACKAGE_PATH}
alias px4flow_launch='roslaunch px4flow_node px4flow.launch'
alias px4_laser_launch='roslaunch ~/ros/quad_control_catkin/src/control/danielpa/launch/quad.launch'
alias laser_test_launch='roslaunch ~/ros/quad_control_catkin/src/control/danielpa/launch/laser_test.launch'
alias shutdown='shutdown -h now'
```

## 6 - yaml.cpp 
https://github.com/jbeder/yaml-cpp

**Why yaml.cpp and not use the rosparam Param Server?** For three main reasons:
 - *Modularity*: yaml.cpp allows to have neatly separated YAML files, each with its own set of parameters.
 - *Local code*: Reading the file is local and uses the library to parse the YAML file. It doesn't spend time fetching things from ROS or acessing a nodehandle.
 - *Simplicity*: There are alot of examples online on how to use this library and they are very straightforward. Also, loading C arrays (not C++ vectors) is easier. To install the library:

```sh
$ git clone https://github.com/jbeder/yaml-cpp
$ cd yaml-cpp/src
$ mkdir build
$ cd ..
$ cmake -G "Unix Makefiles"
$ make
```

## 7 - Optical Flow package
The package should already be in the folders above however you can get it from here aswell:
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
![](http://i.imgur.com/o6DXJMp.jpg)
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

## 13 - QT and GUI (optional)
If you wish to use the camera debug GUI I have made (file blob_camera_debug_qt.cpp) you'll need to install QT like following. If you don't want to use it, please swap the CMakeLists.txt in the cam folder with the old one. On the quadcopter this is not very useful, this is only useful for calibration.
```
file /sbin/init
sudo apt-get install synaptic
sudo apt-get update
sudo apt-get install qt4-dev-tools libqt4-dev libqt4-core libqt4-gui
```
Next get a package called QtSdk-offline-linux-x86_64-v1.2.1.run. Download it from wherever you want ([here](https://packages.kitware.com/item/3734) for example).
```
cd Downloads
sudo -s chmod u+x QtSdk-offline-linux-x86_64-v1.2.1.run
sudo -s ./QtSdk-offline-linux-x86_64-v1.2.1.run -style cleanlooks
```
When you install the Qt SDK you will prompted to select a directory where you want the Qt SDK to reside. Select /opt and your Qt SDK will install into a directory called /opt/QtSDK.
```
sudo -s chmod -R 777 /opt/QtSDK
sudo -s chmod -R 777 /home/"your_user_name"/.config/Nokia
sudo -s nano /etc/profile
```
At the end put:
```
PATH=/opt/QtSDK/Desktop/Qt/4.8.1/gcc/bin:$PATH
export PATH
```
Now source it and test:
```
/etc/profile
qmake -version
```
You should see ```QMake version 2.01a```, followed by ```Using Qt version 4.8.1 in /opt/QtSDK/Desktop/Qt/4.8.1/gcc/lib```

## 14 Optical Flow and Sonar issues and reminders
- There are two ways of using the Optical Flow board, one uses the mavlink protocol and thus communicates to the mavros node launched via PX4 and the other gets serial data directly via USB using a package that is linked above in another point. The one that communicates via mavlink has a different data format and since it needs to be processed before it is in the desired format it is slower, while the direct serial data can reach to up to 200Hz.
- **IMPORTANT**: The OpticalFlow needs to know what USB port it should use. To figure out what port the OF is using simply plug it off and do ```ls /dev```. Then plug it in, and do ```ls /dev``` again and see what has changed. It's probably ```/ttyACM0``` or ```/ttyACM1```. Now, go to the place where you have the package and go to ```px_comm/hardware_interface/launch``` and edit the YAML file there (```px4flow_params.yaml```) to the right port you just found.
- When using the optical flow some of the baseline readings may not be 0 if there are reflections or LEDs flashing. Be careful of this as it will cause velocities to have high values and it will cause instability for the estimators and the x and y values will diverge to infinity.

- Be careful with the referentials don't get confused. PX4 uses NED (North - East - Down), Optical flow uses xyz, but since it's mounted on the quadcopter at 45º and the flying frame is the center of the quadcopter a rotation is applied to obtain the flying frame. Estimator position is (for now) delivered in the flying frame relative to the starting point or a fixed point in the world. Sonar baseline is the minimum operational height - 30 cm, its maximum height is 3m. On Quad1 in our lab, there's an image below that will help you recognize the flying frame:
![](http://i.imgur.com/uDQzhmV.png)

- The sonar doesn't work in all surfaces. Since this is intended for indoor uses, one thing you should be especially wary is *carpets* and *rugs* since their surface do not reflect well. There is a similar effect for the OpticalFlow. Shiny surfaces with very little features aren't ideal. 
- The sonar may have a small height offset of a few mm to the center of mass of the quad.

## 15 Cam issues and reminders
- The cam is on a fixed frame like this ![](http://i.imgur.com/tACf4n3.jpg) and should NOT be moved. It was put this way to have the same xyz frame as the opticalflow/sonar pair and to compare reads.
- The cam should be at least more than 17 cm from a frame and less than 3m. At less than 20 cm the readings may become incoherent. After 3m the blobs may become too small and thus not recognizeable by the algorithm.
- To calibrate you should run ```rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/usb_cam/image```. This calibration uses the cameracalibrator.py script that comes with the package. You'll have to provide it with a checkerboard and with the dimensions of the checkerboard. If you print [this](http://www.imagequalitylabs.com/resources/uploads/product/product_image1338282549.jpg) checkerboard on an A4 paper you'll have the dimensions in the command. For more indepth guide to calibration please see [this](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) link.
Below is an example of a raw image 640x480(YUYV) from the quadcopter in the ISR 6.12 room:
![](http://i.imgur.com/bW0w0V7.png)
For testing, I made a blob frame, which is a quadcopter 3 axis frame with blobs placed similar to the quad leg tips. It also has a place to put an optical flow board, to test its raw data.
![](http://i.imgur.com/Ss37cXs.jpg)

## 16 Laser issues and reminders
- If you go really fast with the quadcopter the laser does not have enough time to update its measurement.
- The flying frame is the following (on our Quad2):
![](http://i.imgur.com/MkfNZTy.jpg)
- Here is how all the frames go together. The x_laser and y_laser in the image below are the frame above presented:
![](http://i.imgur.com/kzwIPOj.png)
- On our quad that has the Hokuyo, the launch file sequence to get the SLAM pose is two step, one to launch the hokuyo_node and another to do a transform: ```px4_laser_launch``` and ```laser_test_launch```. 
- When issuing the last launch file to public the transform between laser and map frames, you may have to re-issue the launch command

- We use the urg/hokuyo node for the **UTM-04LX** laser mounted on our quad ![](https://www.hokuyo-aut.jp/02sensor/07scanner/img/urg_ed_eg.gif)
This [link](http://wiki.ros.org/urg_node) provides a ROS API and driver to set params  (there is an older driver [here](http://wiki.ros.org/hokuyo_node)). For the SLAM approach we use hector_mapping that was made by Stefan Kohlbrecher and Johannes Meyer at Darmstadt, it is maintained, and provides 2D pose estimates at 40Hz. You can find a link [here](http://wiki.ros.org/hector_mapping) that has detailed information.

## 17 NVidia/Quad ssh and wifi
 - Since you login to the NVidia via ssh consider using some sort of graphical program to allow you visualize data onboard such as X. (ssh x for Unix, something like XLaunch with X11 setup in Putty). Also consider using some sort of terminal emulator like [tmux](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-tmux-on-ubuntu-12-10--2) or [terminator](http://technicalworldforyou.blogspot.pt/2012/11/install-terminator-terminal-emulator-in.html). The Wifi dongle used is [this](https://www.asus.com/Networking/USBAC56/) one.
 - QuadBase wireless network is the local network used, either on Quad-Base
  - Find the ip of the quad ```sudo arp-scan --interface=wlan0 --localnet``` (on our local network QuadBase5 the ips are normally the following - they are linked with the quad ID, which is marked on the quadcopter on a white tape)
    - **Quad1**: *Wifi* ```11.0.3.1```, ```11.0.1.5``` or ```11.0.1.51`` *Ethernet* ```10.0.29.251```
    - **Quad2**: *Wifi* ```11.0.3.2``` *Ethernet* ```10.0.20.205```
  - **NEW** To solve some issues of the old procedure a new procedure is in order (by Pedro Agostinho):
    - Jetson network definitions should use DHCP which is the default and it is asked of the users to revert any IP configurations that they have done both on the computers and on the Nvidia boards.
    - Each quad should be paired with a wifi adaptor.
    - The following procedure should be followed for each quad and for the users:
      - IP intervals:
        - 11.0.1.1 a 11.0.1.50 are for the quads (as explained above)
        - The IP's from 11.0.1.51 onwards are reserved for users. Each user should register with the smallest non-used IP
        - The first step is to obtain the MAC address of the Wifi dongle using ```ifconfig```. Obtain also the MAC address of your own computer if you are a user
        - Next head to the browser page of the Quad-Base *http://tplinklogin.net/* user: admin pass: admin
        - Head to *DHCP Options* and then to *Address Reservation*
        - Then all you have to do is associate an IP to that MAC address
   If you are on Windows, ```ping``` these, and you'll probably find the right one). IF you want to connect via ethernet and the ethernet ports are not the listed above, my advice is rather than using a scanner to find the IP which can be time consuming, simply do the following:
    - With the ethernet disconnected, connect the dongle.
    - login via ssh, ```ssh ubuntu@ip```, where ip is one of the above, and accept the ssh key. The password is using the security protocol WPA/WPA2. Now you are connected to the NVidia. If you want to add work to this work please do so in a separate folder placed in ```/home/ubuntu```. To do so, ```mkdir ~/FirstnameLastname```
    **DISCLAIMER**: spreading files outside your directory might lead into accidental deletion of part of your work. Also, make sure that all your files are also stored somewhere outside the device (GitLab/Github/etc...)
    - Insert the ethernet cable on the NVidia. Wait a bit and do ```ifconfig```
    - see the IP address of **eth0**
    - Logout ssh and login with that IP address. Remove the wifi dongle.
    - After you've done all this, please remember to do ```shutdown -h now``` to shut the NVidia down

## 18 - How to fix a broken propeller
First identify the propeller that is broken like so.

![](http://i.imgur.com/oInAP7i.jpg)

Then proceed to remove the nut that is on top of the propeller (use pliers if you need to). Now, gingerly remove the broken propeller by either turning it and making some force upwards so it slides along the bolt. Be careful because it has washer that may fall off. If you want to you may remove the small round plastic piece if it has been damaged. Most of the times this is not the case so you may leave it there. Below is an image with a new propeller, the nut and the small plastic piece. 

![](http://i.imgur.com/wTRXvMOg.jpg)

Introduce the new propeller along the bol with the part that fits in the small plastic piece. Put the washer on top so it looks like this:

![](http://i.imgur.com/TWSB7My.jpg)

**Finally bolt the nut firmly with pliers and you are done.** When using the quadcopter, remain clear of propeller arc and wear eye and hand protection.

## 19 - How to boot and get everything ready
  - Remove all USB connections
  - Connect the battery
  - Wait for NVidia to boot up
  - Connect all USB connections
  - To fly, reboot the PX4 by pressing the small button on the side with a pencil or with the toothpick that is under one of the ESCs
  - Turn the RC on
  - Arm the quad (press the LED)
  - Run your launch files

## 20 - How to charge the batteries and some recommendations:
**How to charge the batteries**:
 - Connections:
   - Turn on the Charging station by flipping the switch on the power supply
   - Connect the red T plug to the charging station
   - Connect the white voltage balancer to the charging station
 - Settings:
   - Select the type of battery, change type with the Batt. Type button and confirm with Enter
   - Confirm the Charging Station is on Charge mode, by using Dec and Inc buttons to select it
   - Press Enter to select the charging amperage and Enter again to confirm
   - Select the end voltage and confirm it with Enter
 - Charging:
   - Press and Hold the Enter Button
   - Check if the selected number of battery’s cell is equal to the number of cell read by the station
   - Press Enter to start the charging process
   - Each press of the Dec. button cycles the status of the charging screen
   - Pressing the Inc. change the screen to display the actual voltage of each battery's cell

**UPDATE** The connectors are no longer T-connectors. Below is an image of the the new connector type:
![](http://i.imgur.com/pvsvXzR.jpg)

**Recommendations**:
 - Don't let the battery voltage go below 12V, because that damages the batteries. If they go below that they can still be charged but will irrevocably have less autonomy.
 - There are **CHARGED** and **DISCHARGED** areas to place the batteries right next to the charger. If you see a battery in the **DISCHARGED** area please recharge them. In our lab there are only 3 batteries and it is inconvenient to find all three discharged. As such, **when you leave the lab please make sure you leave at least 1 charged battery**
 - As a safeguard, make sure the battery really has at least 12V with the voltmeter.
## 21 - How to setup wifi with the USB ASUS Wifi dongles:
 - Download driver from [here](https://github.com/codeworkx/rtl8812au_asus)
 - Add ```CONFIG_PLATFORM_TEGRA_K1 = y``` to the Makefile on platform related; make sure all other platforms are set as ```=n```
 - After I386 make rules, add:
```
ifeq ($(CONFIG_PLATFORM_TEGRA_K1), y)
EXTRA_CFLAGS += -DCONFIG_LITTLE_ENDIAN
ARCH := arm
CROSS_COMPILE := arm-linux-gnueabihf-
KVER := $(shell uname -r)
KSRC ?= /usr/src/linux-headers-$(KVER)
MODDESTDIR := /lib/modules/$(KVER)/kernel/drivers/net/wireless/
endif
```
 - Do ```make```. If it asks to ```make modules_prepare``` on ```/usr/src/linux-headers-KVER```, do so
 - do ```sudo make install```
 - do ```sudo modprobe 8812au```
 - if it gives an error saying it cannot allocate memory, add ```vmalloc=512M``` on ```/boot/extlinux/extlinux.conf``` . Then reboot. If the error persists or if the driver does not work, you may need to reflash TK1 and add ```vmalloc=512M``` on ```/boot/extlinux/extlinux.conf```  to the file right after the first boot, reboot again, and proceed from step one.
 - To reflash the TK1, follow [this](https://gist.github.com/jetsonhacks/2717a41f7e60a3405b34)

 ## 22 - Xbee communication and configuration
 To configure the xbee's you need to follow this installation guide:[http://knowledge.digi.com/articles/Knowledge_Base_Article/HOW-TO-Install-XCTU-in-Linux](http://knowledge.digi.com/articles/Knowledge_Base_Article/HOW-TO-Install-XCTU-in-Linux)
 After that, ```./opt/Digi/XCTU-NG/app``` to run the configure app, set the receiver to admin and the baudrate to 57600. Note that on your computer the Xbee receiver should appear as ```/ttyUSB0```. In doubt, do ```ls /dev```.
 The quadcopters (only Quad1 so far) have an xbee communication system that allows you to see attitude and gyroscope measurements from QGroundControl. The quadcopter has a transmitter and you have a receiver (images below):
 *Receiver - The custom board to interact via FTDI with the USB Port was designed by Pedro Roque*
 ![](http://i.imgur.com/UkLAgyl.jpg)
 *Transmitter - Just attach a simple high gain directional HF antenna*
 ![](http://i.imgur.com/O4P8X3V.jpg)

You'll need to run QGroundControl, click the Q purple icon on the top left corner, and select Comm Links. Then click Add to add the Xbee link:
![](http://i.imgur.com/fgfgmYp.png)
Name it something easy to recognize (XBEE), Type is Serial, set the baud rate to the 57600 and change the Port to /ttyUSB0 (if that's the serial port of the receiver). After that is done, the receiver should be connected to the transmitter on the quadcopter that is paired with the PX4 flight controller to relay information to you.
![](http://i.imgur.com/PwCmXJx.png)

## 23 - Balancing the quads
Whenever you add something to the quads you should make sure that the quad is still balanced. To do so, unscrew the top part of the quadcopter and screw the transparent white platform that has a notch with a fishline tied to it. Tie to a chair or something vertical (you can also hold it but please be very careful) and leveled with the quadcopter access if there's any tilting. Try to wrap wires around the frame legs to balance wire. Below is an image of the platform and the fishline:
![](http://i.imgur.com/GL1qBn8.jpg)