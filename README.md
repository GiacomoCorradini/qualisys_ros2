# qualisys_ros
ROS2 package for Qualisys motion capture system.

outputs `rigid_body/pose`

***
Move this project(folder) under your ros2 workspace `src` folder

`ros2_ws/src/qualisys_ros/`

To build the package, move to `ros_ws/` and do 

`colcon build --packages-select qualisys_ros`

To source the package, move to `ros_ws/`

`. install/setup.bash`

***
To run the node, use 
`ros2 launch qualisys_ros mocap.launch.py`

Change the `--server` (ip addr) and `--rate` (hz) arguments in the launch script to match your system.