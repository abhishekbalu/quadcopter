# quadcopter
## Structure

> This project consists of the ROS files for computer-vision, simulation,
> and controlled flight of a quadcopter using the PX4 autopilot with an NVidia Jetson TK1 as the onboard computer,
> a hector Gazebo simulator and a USB Camera/Hokuyo Laser Node.
> The objective is to evaluate which of the two methods, the USB Camera
> or the Hokuyo Laser is the best for quadcopter pose-estimation.

This project has two folders, rosbuild and catkin. rosbuild is the rosbuild workspace and catkin is the catkin workspace
  - **rosbuild** (rab3D -- estimator, cam -- computer vision, laser -- laser files(launch files/python) asctec -- controller, px-ros-pkg, yaml-cpp packages, simulation -- this package is for simulator, user_control -- package to send commands to the controller)
  - **catkin** (height)

Completed:
  - s̶i̶m̶u̶l̶a̶t̶i̶o̶n̶
  - h̶o̶v̶e̶r̶
  - b̶l̶o̶b̶ d̶e̶t̶e̶c̶t̶i̶o̶n̶
  - d̶i̶s̶t̶a̶n̶c̶e̶ e̶s̶t̶i̶m̶a̶t̶i̶o̶n̶ b̶o̶t̶h̶ o̶n̶ c̶a̶m̶e̶r̶a̶ a̶n̶d̶ l̶a̶s̶e̶r̶
  - s̶o̶m̶e̶ r̶e̶s̶i̶s̶t̶a̶n̶c̶e̶ t̶o̶ e̶n̶v̶i̶r̶o̶n̶m̶e̶n̶t̶ c̶h̶a̶n̶g̶e̶s̶

Future:
  - Use the NVidia CUDA cores for better image processing
  - Improve RGB to HSV conversion
  - Compare cameras