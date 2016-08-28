#! /bin/bash
# /etc/init.d/carobot
source /opt/ros/groovy/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/linaro/rosws/sandbox
export ROS_IP=11.1.1.99
# initialze the pidfile
QUAD_PIDFILE1='/var/run/quad1.pid'
QUAD_PIDFILE2='/var/run/quad2.pid'
ROS_USER=linaro
touch $QUAD_PIDFILE $ROS_USER
/bin/chown $ROS_USER\:$ROS_USER $QUAD_PIDFILE
# roslaunch vars
ROSLAUNCH_BIN="/opt/ros/groovy/bin/roslaunch"
ROSCORE_BIN="/opt/ros/groovy/bin/roscore"
ROSLAUNCH_ARGS1="--pid=$QUAD_PIDFILE1"
ROSLAUNCH_ARGS2="--pid=$QUAD_PIDFILE2"
# set path to launchfile
ROSLAUNCH_FILE1="/home/linaro/rosws/sandbox/internal_ekf/internal_ekf.launch"
ROSLAUNCH_FILE2="/home/linaro/rosws/sandbox/user_control/user_control_all.launch"
# carry out specific functions when asked by the system
case "$1" in
  start)
    echo "Starting quad control services ..."
    exec $ROSLAUNCH_BIN $ROSLAUNCH_ARGS1 $ROSLAUNCH_FILE2 &
    ;;  
  stop)
    kill -2 $(cat $QUAD_PIDFILE1) > /dev/null
    echo -n "" > $QUAD_PIDFILE1
    echo "Stopping quad control services ..."
    ;;  
  restart)
    $0 stop
    sleep 3
    $0 start
    ;;  
  *)  
    echo "Usage: $0 {start|stop|restart}"
    exit 1
    ;;  
esac
exit 0
