# OpenArm Bringup

This package provides launch files to bring up the OpenArm robot system.

## Quick Start

Launch the OpenArm with v1.0 configuration and fake hardware:

```bash
ros2 launch openarm_bringup openarm.launch.py arm_type:=v10 hardware_type:=real
```

## Launch Files

- `openarm.launch.py` - Single arm configuration
- `openarm.bimanual.launch.py` - Dual arm configuration

## Key Parameters

- `arm_type` - Arm type (default: v10)
- `hardware_type` - Use real/mock/mujoco hardware (default: real)
- `can_interface` - CAN interface to use (default: can0)
- `robot_controller` - Controller type: `joint_trajectory_controller` or `forward_position_controller`

## What Gets Launched

- Robot state publisher
- Controller manager with ros2_control
- Joint state broadcaster
- Robot controller (joint trajectory or forward position)
- Gripper controller
- RViz2 visualization

## MoveIt2 Cartesian Target Example

After launching MoveIt2 (`openarm_bimanual_moveit_config`), run:

```bash
ros2 run openarm_bringup move_to_xyz --ros-args \
	-p group_name:=right_arm \
	-p x:=0.35 -p y:=-0.20 -p z:=0.40
```

Then type commands in terminal:

- `x y z` : plan and execute to a new target position
- `s` : stop current trajectory immediately
- `q` : quit the program

You can change target while moving; node will stop current trajectory and re-plan from current state.

Useful parameters:

- `group_name`: `left_arm` or `right_arm`
- `x`, `y`, `z`: target end-effector position in meters
- `ee_link`: optional end-effector link name
- `plan_only`: set `true` to only plan without executing
