# OpenArm Orca Stack

A ROS 2 URDF/Xacro assembly description for dual OpenArm robots equipped with Orca Hands. This repository provides the complete robotic system description including kinematics, visualization, and custom connectors.

## Overview

This stack integrates:
- **Dual OpenArm v10** - Two robotic arms configured for collaborative operation
- **Orca Hands** - Left and right end-effector hands
- **Custom Connectors** - Hardware interface components linking arms to hands

## Repository Structure

```
openarm_orca_stack/
├── assembly_description/          # Top-level robot assembly
│   ├── urdf/assembly.urdf        # Main URDF combining all components
│   ├── launch/view_robot.launch.py
│   └── rviz/default.rviz
├── connectors_description/        # Custom hardware connectors
│   ├── urdf/connector.urdf.xacro
│   └── meshes/connector.stl
├── openarm_description/           # OpenArm robot definitions
│   └── urdf/v10_dual.urdf.xacro
└── orcahand_description/          # Orca Hand end-effectors
    ├── urdf/orcahand_left.urdf.xacro
    └── urdf/orcahand_right.urdf.xacro
```

## Quick Start

### Visualization

Launch RViz to visualize the complete assembly:

```bash
ros2 launch assembly_description view_robot.launch.py
```

### Verification

Run the structure verification script to check all dependencies:

```bash
chmod +x verify_structure.sh
./verify_structure.sh
```

## Dependencies

- ROS 2 (Humble or later recommended)
- RViz2
- xacro

## Components

- **assembly_description**: Main package containing the complete robot assembly
- **connectors_description**: Custom connector hardware models and URDFs
- **openarm_description**: OpenArm v10 dual-arm robot description
- **orcahand_description**: Orca Hand gripper descriptions for left and right hands


