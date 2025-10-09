# Assembly Description Package

This package now provides a flattened OpenArm + OrcaHand assembly built from two
Xacro files:

- `urdf/assembly_flat.urdf.xacro` – the single robot description to load.
- `urdf/assembly_params.xacro` – the shared parameter library imported by the
  flat assembly.

The flat assembly builds the robot from a single root frame (`base_link`) so
that `robot_state_publisher` no longer sees duplicate `world` links. All
component macros accept a prefix parameter to guarantee unique link and joint
names when the left and right arms/hands are instantiated.

## Launching the model

`launch/view_robot.launch.py` has been simplified to run Xacro on the flattened
URDF directly and feed the result to `robot_state_publisher`,
`joint_state_publisher_gui`, and RViz.

## Editing the robot

Update masses, inertias, limits, meshes, and poses in
`urdf/assembly_params.xacro`. The macros in `assembly_flat.urdf.xacro` read
these properties, so tweaking a parameter automatically updates both arms and
hands without touching the geometry definitions.
