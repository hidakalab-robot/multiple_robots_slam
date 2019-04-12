# multiple_robots_slam

now working repositry

This packages need ros-kinetic and are used to do multiple-robots SLAM

<!-- #### "other_robot_name" require robot-number of wanting to know pose (ex. robot1, robot2, etc.) -->
<!-- #### "control_robot_name" require robot-number of wanting to teleoperate (ex. robot1, robot2, etc.) -->

### multi SLAM procedure
#### "robot_name" require robot-number (e.g. robot1, robot2, etc.)

Robot side

1. bringup
```
$ roslaunch multi_turtlebot_launch minimal.launch robot_name:=robot1
```

2. rtabmap (SLAM)
```
$ roslaunch multi_turtlebot_launch mapping.launch robot_name:=robot1
```

3. autonomous exploration
```
$ roslaunch exploration sensor_based_exploration.launch robot_name:=robot1
```

<!-- 
3. share topics and get other-robot pose ( Wait until all masters are displayed in "ROS masters obtained" )
```
$ roslaunch roscore_communication roscore_communication.launch robot_name:=robot1 other_robot_name:=robot2
``` -->

Server side

1. merging maps ( launch after setting each robot's init-pose in "map_merge/launch/map_merge.launch")
```
$ roslaunch map_merge map_merge.launch
```

if you want to use multi roscore system

1. share topics ( launch after setting share topic in "roscore_communication/launch/roscore_communication.launch")
```
$ roslaunch roscore_communication roscore_communication.launch
```

Simulation
1. launch robots with SLAM (launch after setting number of robots and each robot's init-pose in "multi_turtlebot_gazebo/launch/multi_turtlebot_gazebo.launch")
```
$ roslaunch multi_turtlebot_gazebo multi_turtlebot_gazebo.launch
```

- commands for experiment (two robots + server + single master)

  - robot1

    ```
    $ roslaunch multi_turtlebot_launch minimal.launch robot_name:=robot1

    $ roslaunch multi_turtlebot_launch mapping.launch robot_name:=robot1

    $ roslaunch exploration sensor_based_exploration.launch robot_name:=robot1
    ```

  - robot2

    ```
    $ roslaunch multi_turtlebot_launch minimal.launch robot_name:=robot2

    $ roslaunch multi_turtlebot_launch mapping.launch robot_name:=robot2

    $ roslaunch exploration sensor_based_exploration.launch robot_name:=robot2
    ```
  - server

    ```
    $ roslaunch map_merge map_merge.launch
    ```
