# robotarm_ws

Apply forward kinematics and inverse kinematics using the robot arm's URDF file. 

This repo is made specifically for Yahboom DOFBOT 6-DoF robot arm but could work with other robot arms. Save the URDF file of your robot arm into the URDF folder and change the urdf file name at line 67 in robotarm_ws/src/kinematics_pkg/scripts/ik_solver.py

Launch input.py, ik_solver.py and mover.py in the script folder simulatenously.

The function of these nodes are

input.py - user input values of either coordinates or servo angles. based on the length of the input, the program will either send to ik_solver.py or mover.py

ik_solver.py - receives the x,y,z coordinates the user inputted in input.py then solves the inverse kinematics to get the angle needed to get to the target location

mover.py - moves the robotic arm

This program uses ROS Noetic on Linux 20.04.

This repo was created to build the project below for my final year project. Colour sorting robot arm for pick and place applications. Built the software and programs from the group up. Embed link can be found below.

https://youtu.be/7RF_a9DTxpA

