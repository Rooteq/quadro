# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 quadruped robot package named "quadro" that implements a 12-DOF (3 DOF per leg) robotic quadruped with simulation support in Gazebo and real hardware control capabilities. The robot uses Pinocchio for advanced dynamics and kinematics calculations.

## Build System

- **Package manager**: ROS2 (ament_cmake)
- **Build tool**: colcon
- **Dependencies**: Requires Pinocchio library for dynamics/kinematics

### Build Commands
```bash
# Build the package
colcon build --packages-select quadro

# Source the setup
source install/setup.bash

# Build with debug info
colcon build --packages-select quadro --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Launch Files and Simulation

### Main Launch Commands
```bash
# Launch full robot simulation in Gazebo with RViz
ros2 launch quadro robot_gazebo_launch.py

# Launch single leg for testing
ros2 launch quadro leg_gazebo.py

# Launch real hardware control
ros2 launch quadro one_motor_real.py
```

### Controller Commands
```bash
# Load joint state broadcaster
ros2 control load_controller --set-state active joint_state_broadcaster

# Load effort controller for torque control
ros2 control load_controller --set-state active effort_controller

# Send torque commands (example from commands.txt)
ros2 topic pub --once /effort_controller/commands std_msgs/msg/Float64MultiArray "data: [10.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 0.0]"
```

## Code Architecture

### Key Executables
1. **trajectory_publisher** (`src/simple_trajectory_publisher.cpp`) - Basic trajectory generation node
2. **torque_controller** (`src/torque_controller.cpp`) - Advanced torque control using Pinocchio for computed torque control

### Robot Description
- **Base file**: `description/robot.xacro` - Full quadruped robot definition with 4 legs (12 joints total)
- **Leg module**: `description/leg.xacro` - Single 3-DOF leg definition
- **Joint naming**: Each leg has joints named as `{leg}_m{1,2,3}_s{1,2,3}` where leg = {br, bl, fr, fl}
- **End effectors**: Four feet with contact points for ground interaction

### Hardware Interface
- **Hardware plugin**: `hardware/hw_actuators.cpp` - Custom hardware interface for motor control
- **Motor drivers**: `hardware/cybergear_driver_core/` - Core drivers for CyberGear motors
- **Real hardware**: Uses CAN bus communication via ros2_socketcan

### Controller Configuration
- **Torque control**: `config/torque_controllers.yaml` - Main control configuration
- **Joint position**: `config/joint_position_controllers.yaml` - Position control setup
- **Bridge params**: `config/bridge_params.yaml` - Gazebo-ROS bridge configuration

## Development Workflow

### Testing Commands
```bash
# Run linting (if available)
ament_lint_auto

# Test single motor control
ros2 run quadro torque_controller
```

### Simulation Modes
- Set `sim_mode:=true` in launch files for Gazebo simulation
- Set `sim_mode:=false` for real hardware control
- Robot spawns at position (0.5, 0.5, 0.07) in simulation

### Key Topics
- `/effort_controller/commands` - Send torque commands to all 12 joints
- `/joint_states` - Joint state feedback
- Controller manager topics for loading/unloading controllers

## Important Notes

- Robot uses Pinocchio for dynamics calculations - ensure it's installed (`find_package(pinocchio REQUIRED)`)
- URDF files are generated from Onshape CAD models
- Hardware interface expects CyberGear motors with CAN communication
- Each leg has 3 DOF: hip (m1), thigh (m2), shin (m3) joints
- Mimic joints ensure proper linkage coupling between joints 2 and 3 on each leg