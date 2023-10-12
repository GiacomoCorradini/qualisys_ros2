# qualisys_ros2

ROS2 package for Qualisys motion capture system ([original repository](https://github.com/zp-yang/qualisys_ros)).

the output is a ros2 topic: `rigid_body/pose`

## Build the package

Build the package:

```bash
colcon build --packages-select qualisys_ros
```

To source the package:

```bash
. install/setup.bash
```

## How to use the node

To run the node, type: `ros2 launch qualisys_ros mocap.launch.py`

Change the `--server` (ip addr) and `--rate` (hz) arguments in the launch script to match your system.