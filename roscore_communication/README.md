# roscore_communication

## This package is used to share independent rosmasters topics

### "robot_name" require robot-number (ex. robot1, robot2, etc.)
### "other_robot_name" require robot-number of wanting to know pose (ex. robot1, robot2, etc.)

* share topics and get other-robot pose
```
$ roslaunch roscore_communication roscore_communication.launch robot_name:=robot1 other_robot_name:=robot2
```
