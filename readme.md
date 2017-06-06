mapping2d_orbslam
-----
订阅orbslam2输出定位信息tf构建2d地图
## Prerequisites
### turtlebot
参考：https://github.com/rongbohou/cvte-Turtlebot

### cvte-ORBSLAM2

参考：https://github.com/rongbohou/cvte-ORBSLAM2

## Building

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

git clone https://github.com/rongbohou/cvte-mapping2d.git
cd ~/catkin_ws
catkin_make
```

## Usage
```
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation mapping_orbslam_rplidar.launch

cd ~/rongbo/catkin_ws/orbslam2
rosrun orbslam2 localization /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/ORBvoc.bin /home/bobo/rongbo/catkin_ws/src/my_orbslam/config/kinect2_qhd.yaml true true
```
