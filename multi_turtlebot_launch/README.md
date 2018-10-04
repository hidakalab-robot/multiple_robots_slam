# muilti_turtlebot_launch

## This package is used to launch robot

### "robot_name" require robot-number (ex. robot1, robot2, etc.)

* bringup

```
$ roslaunch multi_turtlebot_launch minimal.launch robot_name:=robot1
```

* rtabmap

```
$ roslaunch multi_turtlebot_launch mapping.launch robot_name:=robot1
```

* teleop

```
$ roslaunch multi_turtlebot_launch teleop.launch robot_name:=robot1
```
