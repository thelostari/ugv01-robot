# UGV01 Robot Pre-Navigation Preparation

This repository contains a few ROS2 packages for the foundation of navigation of the [UGV01 robot](https://www.waveshare.com/wiki/UGV01) built by Waveshare. It has:
- keyboard control of the UGV01 robot over wifi
- simplified URDF description of the robot
- LiDAR controls for the [RPLIDAR A1](https://www.slamtec.com/en/lidar/a1) by SLAMTEC (see sllidar_ros2 directory for the specific README file with instructions, or SLAMTEC's own [repository](https://github.com/Slamtec/sllidar_ros2) in case of updates)

## How To Use
NOTE: This code was developed with ROS2 Humble in Ubuntu 22.04. It was completely tested in ROS2 Humble in Ubuntu 22.04, and partially tested in ROS2 Jazzy in Ubuntu 24.04 running on a Raspberry Pi 5. It should be compatible with other ROS2 and Ubuntu versions, but some modifications may be needed. When using terminals for the instructions below, there may be errors (especially iwth RViz) caused by environment variables set by vscode, so prefer using system terminals outside of vscode to avoid that.

1. Add these packages to the src folder of a ROS2 workspace. If needed, check ROS2 documentation on how to [install it](https://docs.ros.org/en/humble/Installation.html), [configure the environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html), and [create a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).
2. Use [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) to build the workspace. There should be no errors, only warnings from sllidar.

Explained below are the different ways these packages can be used.

---
### Keyboard Control Only
If you only want to control the robot via keyboard over wifi.

1. Turn on the robot
2. Connect your computer to the robot's wifi (UGV01_BASE)
3. Open a terminal, move to the workspace root and source the ROS2 installation as well as the workspace itself
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
6. With the terminal running teleop_twist_keyboard in focus, follow the key bindings to move the robot

---
### Visualize Robot URDF in RViz Only
If you only want to see the URDF in RViz for modifying and testing.

1. Open a terminal, move to the workspace root and source the ROS2 installation as well as the workspace itself
```bash
cd <workspace_name>
source /opt/ros/<distro>/setup.bash
source install/setup.bash
```
2. Run the view_ugv01.launch.py launch file
```bash
ros2 launch ugv01_description view_ugv01.launch.py
```

#### Without the launch file
You can also do this without using the launch file.

1. Open two terminals, move to the workspace root and source the ROS2 installation as well as the workspace itself in each of them
```bash
cd <workspace_name>
source /opt/ros/<distro>/setup.bash
source install/setup.bash
```
2. Run the robot state publisher in the first terminal (replace the path with yours)
```bash
ros2 run robot_state_publisher robot_state_publisher \
--ros-args \
-p robot_description:="$(< ~/ugv01_ws/src/ugv01_description/src/description/ugv01_description.urdf)"
```
3. Run the joint state publisher gui in the second terminal (replace the path with yours)
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui \
--ros-args \
-p robot_description:="$(< ~/ugv01_ws/src/ugv01_description/src/description/ugv01_description.urdf)"
```
4. Open another terminal and run rviz2
```bash
rviz2
```
Inside RViz:

5. Set Fixed Frame to base_link
6. Click Add and choose RobotModel
7. Set RobotModel/DescriptionTopic to /robot_description
8. Optionally, click Add and choose TF to see axes

---
### Keyboard Control of the Robot and Simulation in RViz
If you want to control the robot via keyboard over wifi and see the simulation in RViz.

NOTE: The odometry is not calibrated correctly, so there is a big difference between the movement of the robot in real life and in the simulation.

1. Turn on the robot
2. Connect your computer to the robot's wifi (UGV01_BASE)
3. Open a terminal, move to the workspace root and source the ROS2 installation as well as the workspace itself
```bash
cd <workspace_name>
source /opt/ros/<distro>/setup.bash
source install/setup.bash
```
4. Run the test_ugv01_odom.launch.py launch file
```bash
ros2 launch ugv01_http_driver test_ugv01_odom.launch.py
```
5. In another terminal, source the ROS2 installation again and run [teleop_twist_keyboard](https://docs.ros.org/en/ros2_packages/humble/api/teleop_twist_keyboard) (you may need to install this package first)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
6. With the terminal running teleop_twist_keyboard in focus, follow the key bindings to move the robot
7. Additionally, you can see the odometry readings by opening a third terminal, moving to the workspace root, sourcing the ROS2 installation as well as the workspace itself, and running the ROS2 topic echo tool
```bash
cd <workspace_name>
source /opt/ros/<distro>/setup.bash
source install/setup.bash

ros2 topic echo /odom
```

## Potentially Useful Resources
### UGV01 Robot
- https://github.com/waveshareteam
- https://www.waveshare.com/product/robotics/mobile-robots/ugv01.htm
- https://www.waveshare.com/wiki/UGV01

### LiDAR
- https://github.com/Slamtec
- https://github.com/Slamtec/sllidar_ros2
- https://www.youtube.com/watch?v=ao13F-L_TAI
- https://www.youtube.com/watch?v=OSoMSVry-8E

### ROS2
- https://docs.ros.org/en/humble/Tutorials.html
- https://github.com/ros
- https://github.com/ros/ros_tutorials/tree/humble

### ROS2 Nav
- https://docs.nav2.org/setup_guides/index.html
- https://github.com/ros-navigation
- https://docs.ros.org/en/ros2_packages/humble/api/slam_toolbox/
