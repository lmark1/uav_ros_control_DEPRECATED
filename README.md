# UAV ROS Control

This repository contains UAV control algorithms for use with the Mavros interface.

## Usage
```bash
export UAV_NAMESPACE=red; roslaunch uav_ros_control pid_carrot.launch
```

Argument **control_type** denotes the control program used:
* pid_cascade_node - Ardupilot compatible control using roll-pitch-yaw-thrust commands
* pid_cascade_node_yawrate - Ardupilot compatible control using roll-pitch-yawrate-thrust commands
* pid_cascade_node_px4 - PX4 compatible control using roll-pitch-yaw-thrust commands
* pid_cascade_node_px4_yawrate - PX4 compatible control using roll-pitch-yawrate-thrust commands

## About

* Code documentation can be found [here](https://lmark1.github.io/uav_ros_control)