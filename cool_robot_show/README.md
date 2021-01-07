# Cool Robot Show
* This is a package for cool robot show.
* Be familiar to the 2-D lidar(Rplidar).
* Use the visual information and the inertial measurement to build a map for the environment (implement in the simulation).
* Follow the instruction [here](https://hackmd.io/VlPtlzJtRN6Yd4gUQ33pJw).

## Script on Arduino
Run arduino/cool_robot_arduino/cool_robot_arduino.ino on Arduino.

## Launch file on Raspberry Pi3
```
roslaunch cool_robot_show cool_robot_launch.launch
```

## View in Rviz
```
cd rviz/
rviz -d rplidar.rviz
```
