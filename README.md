# follow : ROS package
a ROS package for color based following

## Installation


Follow the install [guide](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/) for turtlebot3 PC setup.

### Install Ubuntu on Remote PC.

Install ROS on Remote PC

Install turtlebot3 Dependent ROS Packages



install [gazebo](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo) from turtlebot3 PC setup guide.



## Setup


```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/zeged85/opencv_track3.git
$ cd ~/catkin_ws && catkin_make
```


## Bringup
choose simulator or robot

### terminal 1

simulator
```bash
roslaunch follow follow_sim.launch 
```
or normal robot bringup.

### terminal 2

launch controller
```bash
roscd follow
cd src
python ./follow3.py
```

### terminal 3

launch GUI

```bash
roscd follow
cd src
python ./auto_ros_commands.py 
```







```python
import foobar

foobar.pluralize('word') # returns 'words'
foobar.pluralize('goose') # returns 'geese'
foobar.singularize('phenomena') # returns 'phenomenon'
```


