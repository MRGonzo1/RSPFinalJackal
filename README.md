# RSP Final Project ROS2 Foxy Jackal

# Compile Instructions

Instructions for Running

```bash
mkdir -p ~/ros_jackal/src
cd ~/ros_jackal/src
git clone https://github.com/MRGonzo1/RSPFinalJackal.git
rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y
export IGNITION_VERSION=fortress
colcon build
```

To run the Ignition Simulation

```bash
ros2 launch jackal_gazebo jackal_world.launch.py
```
Controling the robot, in a new terminal

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped
```

# Usage

## Video + Pictures

![](img/ign_ros2_control.gif)

![](img/diff_drive.gif)


## Running



