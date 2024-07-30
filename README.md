# Robot Arm Workspace

This repository contains the implementation of forward kinematics and inverse kinematics using the robot arm's URDF file. Although specifically designed for the Yahboom DOFBOT 6-DoF robot arm, it can be adapted for other robot arms by saving the URDF file in the URDF folder and updating the file name in `robotarm_ws/src/kinematics_pkg/scripts/ik_solver.py` at line 67.

## Features

- **Forward Kinematics**: Calculate the position of the robot arm's end-effector based on the given joint angles.
- **Inverse Kinematics**: Determine the necessary joint angles to place the end-effector at a desired position.

## Usage

1. Save your robot arm's URDF file into the `URDF` folder.
2. Update the URDF file name in `ik_solver.py` at line 67.
3. Launch `input.py`, `ik_solver.py`, and `mover.py` in the `scripts` folder simultaneously.

## Nodes

- **input.py**: Accepts user input of either coordinates or servo angles. Based on the length of the input, it sends the data to either `ik_solver.py` or `mover.py`.
- **ik_solver.py**: Receives the x, y, z coordinates from `input.py` and solves the inverse kinematics to determine the necessary joint angles to reach the target position.
- **mover.py**: Executes the movements of the robotic arm based on the calculated angles.

## Requirements

- ROS Noetic on Linux 20.04

## Project Context

This repository was created for a final year project involving a color sorting robot arm for pick-and-place applications. The software and programs were developed from scratch to achieve this functionality.

[Watch the project in action](https://youtu.be/7RF_a9DTxpA)
