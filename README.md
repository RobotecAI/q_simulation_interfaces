# QSimulationInterfaces

A Gui Tool to change the simulation parameters using a GUI interface.
It utilizes https://github.com/ros-simulation/simulation_interfaces to change the simulation parameters.

# Prerequisites

```shell
sudo apt install ros-$ROS_DISTRO-simulation-interfaces libqt5-dev
```

# Building

```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/RobotecAI/q_simulation_interfaces.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

The tool is available in two flavors:

- A standalone GUI application that can be launched as ROS 2 node:
```shell  
ros2 run q_simulation_interfaces q_simulation_interfaces_standalone
```
- A Rviz 2 panel plugin that can be launched from Rviz 2 by clicking on the `Panels` menu and selecting `Add New Panel...`
  and then selecting `SimulationInterfacesPanel` from 'q_simulation_interfaces' package.