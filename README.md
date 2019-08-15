# follow : ROS package
a ROS package for color based following

## Installation


### 1. Follow the install [guide](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/) for turtlebot3 PC setup.

1.1. Install Ubuntu on Remote PC.

1.2. Install ROS on Remote PC

1.3. Install turtlebot3 Dependent ROS Packages



### 2. install [gazebo](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo) from turtlebot3 PC setup guide.



## Setup


```bash
cd ~/catkin_ws/src/
git clone https://github.com/zeged85/opencv_track3.git
cd ~/catkin_ws && catkin_make
cp ~/catkin_ws/src/follow/misc/plannar_mover.cpp ~/catkin_ws/src/plannar_mover/src/

```


## Bringup
choose simulation or robot

### terminal 1 - rosCore
optional

alternatively u can run normal robot bringup.

simulator
```bash
roslaunch follow follow_sim.launch 
```

### terminal 1.1 - teleop

```bash
roslaunch follow linear_teleop_key.launch
```





### terminal 2 - Control

launch controller
```bash
roscd follow
cd src
python ./follow3.py
```

### terminal 3 - GUI

launch GUI

```bash
roscd follow
cd src
python ./auto_ros_commands.py 
```







