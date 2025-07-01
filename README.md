# QSimulationInterfaces

A Gui Tool to change the simulation parameters using a GUI interface.
It utilizes https://github.com/ros-simulation/simulation_interfaces to change the simulation parameters.

# Prerequisites

I don't know. ROS 2 Humble and libqt5-dev at least.

# Building

```bash
mkdir -p /tmp/ros2_ws/src
cd /tmp/ros2_ws/src
git clone https://github.com/ros-simulation/simulation_interfaces.git
git clone https://github.com/michalpelka/q_simulation_interfaces.git
cd ..
colcon build --symlink-install
source install/setup.bash
ros2 run q_simulation_interfaces q_simulation_interfaces
```


