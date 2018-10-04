# multiple_robots_slam

## This packages need ros-kinetic and are used to do multiple-robots SLAM

#### "robot_name" require robot-number (ex. robot1, robot2, etc.)
#### "other_robot_name" require robot-number of wanting to know pose (ex. robot1, robot2, etc.)

- multi SLAM procedure

1. bringup

```
$ roslaunch multi_turtlebot_launch minimal.launch robot_name:=robot1
```

2. rtabmap

```
$ roslaunch multi_turtlebot_launch mapping.launch robot_name:=robot1
```

3. share topics and get other-robot pose ( Wait until all masters are displayed in "ROS masters obtained" )
```
$ roslaunch roscore_communication roscore_communication.launch robot_name:=robot1 other_robot_name:=robot2
```

4. merging maps ( launch after setting each robot's init-pose to "map_merge/launch/map_merge.launch")

```
$ roslaunch map_merge map_merge.launch robot_name:=robot1
```
