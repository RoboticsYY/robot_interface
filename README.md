# robot_interface

ROS2 package to use robot native interface

## Install

Install dependency **ur_modern_driver**:

```shell
git clone -b libur_modern_driver https://github.com/RoboticsYY/ur_modern_driver.git
cd libur_modern_driver
mkdir build && cd build
cmake .. && sudo make install
```

Install dependency **ros2_ur_description**:

```shell
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/RoboticsYY/ros2_ur_description.git
cd .. && colcon build
```

Install **robot_interface**:

The installation should refer to the installation of **ros2_grasp_library**.

## Launch

Launch the UR robot control test executable:

```shell
ros2 launch robot_interface ur_test.launch.py
```

Launch the Rivz2 display:

```shell
ros2 launch ur_description view_ur5_ros2.launch.py
```
