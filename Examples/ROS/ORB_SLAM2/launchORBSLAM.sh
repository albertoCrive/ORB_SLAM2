#!/bin/bash

# TODO add custom catkin_ws path, currently assuming it's in ~

# change these paths if needed!
ORB_SLAM_ROOT=/home/crivella/programmi/ORB_SLAM2

#--disable-factory needed for getting pid of background terminal

gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscore" ' &

sleep 4

gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscd usb_cam && cd launch/ && roslaunch usb_cam-test.launch " ' &

# launch original orb slam 2 ros package
# gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/crivella/programmi/ORB_SLAM2/Examples/ROS && cd /home/crivella/programmi/ORB_SLAM2 && echo ${ROS_PACKAGE_PATH} && rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/KITTI00-02.yaml" ' &

COMMAND='/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${ORB_SLAM_INSTALL_ROOT}/Examples/ROS && cd ${ORB_SLAM_INSTALL_ROOT} && rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/KITTI00-02.yaml" '
ORB_SLAM_INSTALL_ROOT=${ORB_SLAM_ROOT}  gnome-terminal --disable-factory -e "${COMMAND}" &

trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT

