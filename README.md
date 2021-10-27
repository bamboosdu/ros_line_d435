# ROS_LINE_D435
## 构成说明

1、Realsense_lines_detection-master：寻线功能包，其中follower/src/中为源码

2、robot_vision：二维码功能包，其中（1）./camera_calibration.yaml为对相机进行但目标定的参数。（2）./launch/usb_cam_with_calibration.launch文件中设置所使用的相机ID。

## agx部署寻线/二维码识别功能

1、agx部署ros环境、opencv环境(agx自带)。

2、部署二维码功能包环境
```
$ sudo apt-get install ros-melodic-ar-track-alvar
$ sudo apt-get install ros-melodic-usb-cam
```

## 编译程序

1、创建ros工作区，和src文件，将源码导入src文件夹中；
```
$ mkdir -p ros_line_QRcode
$ mkdir src && cd src
```
2、编译源码
```
$ cd ..
$ catkin_make
```

## 运行程序

1、开启终端，启动ros环境
```
$ roscore
```

2、开启新终端，进入ros工作区
```
$ source devel/setup.bash
```

3、开启新终端，进入ros工作区，启动相机
```
$ roslaunch robot_vision usb_cam_with_calibration.launch
```

4、开启新终端，进入ros工作区，启动二维码识别
```
$ roslaunch robot_vision ar_track_camera.launch
```

5、开启新终端，进入ros工作区，启动巡线识别
```
rosrun follower follower_node
```

