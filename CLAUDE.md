# q_simulation_interfaces

A ROS 2 package that provides a Qt-based GUI panel for controlling simulation parameters and entities. This package is designed to work as both an RViz2 plugin and a standalone application.

## Project Overview

This is a Qt5/C++ ROS 2 package that creates a GUI interface for the `simulation_interfaces` package. It provides comprehensive simulation control capabilities including entity spawning, state management, and simulation stepping.

## Architecture

- **Main Library**: `libq_simulation_interfaces.so` - RViz2 plugin library
- **Standalone Executable**: `q_simulation_interfaces_standalone` - Independent Qt application
- **Core Component**: `SimulationPanel` class - Main GUI controller

## Key Files Structure

### Source Files
- `src/simulation_panel.cpp` - Main panel implementation with all GUI logic
- `src/standalone_main.cpp` - Standalone application entry point
- `include/q_simulation_interfaces/simulation_panel.hpp` - Panel header
- `src/service.h` - Template-based ROS 2 service wrapper utility
- `src/stringToKeys.h` - String/key mapping utilities for simulation enums
- `src/vector_utils.hpp` - Vector utility functions for Qt/ROS conversions
- `src/sim_widget.ui` - Qt Designer UI file

### Configuration Files
- `package.xml` - ROS 2 package manifest with Qt5 dependencies
- `CMakeLists.txt` - Build configuration with Qt5 and ROS 2 integration
- `q_simulation_plugins.xml` - RViz plugin description for pluginlib

## Dependencies

### ROS 2 Dependencies
- `rclcpp` - ROS 2 C++ client library
- `rclcpp_action` - ROS 2 action client/server
- `simulation_interfaces` - Main simulation service/message definitions
- `tf2` - Transform library for quaternion operations
- `pluginlib` - Plugin management for RViz integration
- `rviz_common` - RViz panel base classes

### System Dependencies
- Qt5 (Widgets module)
- `qt5-qmake` and `qtbase5-dev` packages

## Functionality

### Entity Management
- **Spawn Entities**: Create new simulation entities from available spawnables
- **Get Entities**: List all current entities in simulation
- **Entity State**: Get/set position, orientation, and velocity of entities
- **Despawn**: Remove entities from simulation

### Simulation Control
- **Step Simulation**: Advance simulation by specified number of steps (service or action)
- **Reset Simulation**: Reset simulation with different scopes
- **Get/Set Simulation State**: Control simulation running state
- **Get Features**: Query simulator capabilities

### Service Integration
All functionality is accessed through ROS 2 services defined in `simulation_interfaces`:
- `/get_spawnables`, `/spawn_entity`, `/delete_entity`
- `/get_entities`, `/get_entity_state`, `/set_entity_state`
- `/get_simulation_features`, `/reset_simulation`
- `/get_simulation_state`, `/set_simulation_state`
- `/step_simulation`, `/simulate_steps` (action)

## Build Instructions

```bash
# In ROS 2 workspace
colcon build --packages-select q_simulation_interfaces
source install/setup.bash

# Run as RViz plugin
rviz2

# Run standalone
ros2 run q_simulation_interfaces q_simulation_interfaces_standalone
```

## Usage Modes

### RViz2 Plugin
- Integrates as a panel in RViz2
- Access via Panels menu → Add New Panel → q_simulation_interfaces → SimulationPanel

### Standalone Application
- Independent Qt application
- Full functionality without RViz2 dependency
- Useful for dedicated simulation control interfaces

## Code Patterns

### Service Template Pattern
The `Service<T>` template class in `service.h` provides:
- Unified service calling interface
- Automatic timeout handling
- Error checking and user feedback via QMessageBox
- Special handling for different service types

### Qt Integration
- Uses Qt5 widgets with Qt Designer UI files
- Signal/slot connections for GUI events
- Qt/ROS data type conversions (quaternions, vectors)
- Thread-safe action client handling

### Plugin Architecture
- Implements `rviz_common::Panel` interface
- Uses pluginlib for dynamic loading
- Exports plugin via `PLUGINLIB_EXPORT_CLASS` macro

## Development Notes

- Uses C++17 standard
- CMake handles Qt5 MOC (Meta-Object Compiler) automatically
- Thread management for long-running actions (simulation stepping)
- Comprehensive error handling with user-friendly dialogs
- Maintains GUI state synchronization with simulation state

## Git Status
Current branch: `mp/rviz2` (working on RViz2 integration)
Main branch: `master`
Recent commits focus on UI improvements and functionality additions.