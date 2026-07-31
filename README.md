# UGV01 Robot Pre-Navigation Preparation

This repository contains a few ROS2 packages for the foundation of navigation of the [UGV01 robot](https://www.waveshare.com/wiki/UGV01) built by Waveshare. It has:
- keyboard control of the UGV01 robot over wifi
- simplified URDF description of the robot
- LiDAR controls for the [RPLIDAR A1](https://www.slamtec.com/en/lidar/a1) by SLAMTEC (see sllidar_ros2 directory for the specific README file with instructions, or SLAMTEC's own [repository](https://github.com/Slamtec/sllidar_ros2) in case of updates)

## How To Use
This code was developed with ROS2 Humble in Ubuntu 22.04. It was completely tested in ROS2 Humble in Ubuntu 22.04, and partially tested in ROS2 Jazzy in Ubuntu 24.04 running on a Raspberry Pi 5. It should be compatible with other ROS2 and Ubuntu versions, but some modifications may be needed.

1. Add these packages to the src folder of a ROS2 workspace. If needed, check ROS2 documentation on how to [install it](https://docs.ros.org/en/humble/Installation.html), [configure the environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html), and [create a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

2. Use [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) to build the workspace. There should be no errors, only warnings from sllidar.

---
### Keyboard Control Only
If you only want to control the robot via keyboard over wifi.

1. Turn on the robot
2. Connect your computer to the robot's wifi (UGV01_BASE)
3. Move to the workspace root and source the ROS2 installation (this step has probably already been done before building the workspace) as well as the workspace itself
```bash
cd <workspace_name>
source /opt/ros/<distro>/setup.bash
source install/setup.bash
```
4. Run the robot_comm_node.py node (in case of change, the correct name can be found or altered in setup.py)
```bash
ros2 run ugv01_http_driver ugv01_http_driver
```
5. In another terminal, source the ROS2 installation again and run [teleop_twist_keyboard](https://docs.ros.org/en/ros2_packages/humble/api/teleop_twist_keyboard) (you may need to install this package first)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
With the terminal running teleop_twist_keyboard in focus, follow the key bindings to move the robot.

---
### Visualize Robot URDF in RViz Only
If you only want to see the URDF in RViz for modifying and testing.
