#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")
docker build --build-arg UID="$uid" --build-arg GID="$gid" --build-arg ROS_DISTRO=humble -t realsense_driver/ros:humble .

echo "Run Container"
xhost + local:root
docker run --name realsense_driver --privileged -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/neobotix/.Xauthority:/home/docker/.Xauthority -v /dev:/dev --net host --rm realsense_driver/ros:humble
# -v $PWD/src/realsense-ros:/home/docker/ros2_ws/src
