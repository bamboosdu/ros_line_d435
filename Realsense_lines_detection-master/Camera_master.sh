#!/bin/bash

sudo rmmod uvcvideo
sudo modprobe uvcvideo nodrop=1 timeout=5000

source /home/upsquared/catkin_ws6/devel/setup.bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
