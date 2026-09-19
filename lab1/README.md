# Lab 1 - Setting up IASYS environment (Ubuntu 24.04 LTS, ROS2, Autoware.Auto) and exploring an autonomous system

## Table of contents

- [What was done and how](#what-was-done-and-how)
- [Lab 1 Group Questions](#lab-1-questions)
- [Think in Sense-Plan-Act table](#think-in-sense-plan-act)
- [Failure experiment](#failure-experiment) 

## What was done and how
In this lab we followed the instructions in [lab1 pdf](IASYS-LAB1-2026-v3.0.pdf)

- Ended up having ubuntu 24.04 LTS running on a VM (using virt-manager on a fedora host)
    - downloaded the iso at [iso link](https://old-releases.ubuntu.com/releases/noble/ubuntu-24.04.1-desktop-amd64.iso)
    - Used the iso to create a ubuntu VM on virt-manager
    - In addition to the lab instructions, I did the following configurations:
        - Changed firmware to UEFI(OVMF_CODE.fd variant)
        - Turned on 3D Acceleration and OpenGL

- Installed docker on the Ubuntu VM
- Installed ADE (for autoware.auto reference environment) on the Ubuntu VM
- Installed Autoware.Auto on the Ubuntu VM
- Set up a shared folder in the VM and copied lab1 resources to it
- Checked, ran and evaluated the autonomous system

## Lab 1 Questions 

Q1: What information must the system know about the vehicle itself

A: The system must know the dimensions of the vehicle (geometric footprint), position and orientation, speed (longitudinal speed and yaw rate), kynematics and dynamic constraints and its current actuation state.

Q2: What information must it (the system I suppose) know about the environment?

A: The system must known the bondaries of the environment and its frame, the dimensions, the objects in the environment (static and dynamic objects), the obstacle objects geometry and position (their occupancy), the velocity, position and orientation of the dynamic objects and also the goal position. 

Q3:  Which part of the behaviour appears to correspond to planning?

A:The moving of the vehicle towards the goal without colliding with obstacles, which corresponds to the green path from the current position to the goal.

Q4: Which part corresponds to actuation?

A:The vehicle executing the speed and steering commands

Q5: Which information appears to change continuously while the vehicle moves?

A:During the run, the vehicle constantly changed the path to the goal (the green line) and the vehicle position and orientation changed accordingly.

Q6: Does the vehicle appear to be following a pre-recorded motion, or reacting to a goal? Explain your reasoning

A: The vehicle does not appear to be following a pre-recorded motion, as stated in the previous answer it reacts to the goal and updates the path along the way, it also can also react to the change of the goal mid course.

## Think in Sense-Plan-Act

| Function | What you observed | What information is probably required |
|---|---|---|
| Sense / state acquisition | The vehicle marker moves and turns continuously in RViz2, the blue driven trajectory grows behind it, and the static obstacles and map boundary remain fixed throughout the run. | Vehicle pose (position and orientation) and velocity (longitudinal speed and yaw rate), the map extent and its frame, occupancy of free and blocked space, obstacle geometry and position, and the goal pose set with the 2D Goal Pose tool. |
| Plan | A green path appears from the current vehicle position to the goal, curving around the obstacles rather than crossing them, and a new path is produced when a new goal is selected. The path changes before the vehicle changes its motion. | Current vehicle pose, goal pose, occupancy and obstacle geometry, and the vehicle footprint and kinematic constraints so that the path is collision free and physically executable. |
| Act / control | The vehicle advances along the green path and changes heading as it turns, following the planned route until it reaches the goal. | The planned path as reference, the current pose and velocity as feedback, the current actuation state, and the dynamic limits on acceleration, deceleration and steering. |

## Failure Experiment

Relaunching the demonstration with the controller disabled:
> ros2 launch iasys_demo demo.launch.py controller_enabled:=false

Does a valid path still exist?
Yes a path to the goal avoiding the obstacles still exist (the green path)

Does the vehicle move?
No, the vehicle is stopped

What capability has been lost?
The actuation capability, the vehicle cannot act to follow the path to the goal

Is the system still autonomous merely because some autonomous functions remain active?
Without an action and just the plan we cannot say that the system is still autonomous since it is not capable of acting autonomously according to the plan function that still remains active

What should a safety-aware system do when an essential component fails?
A safety-aware system must detect the failure, announce it, should have fallback mechanisms and  enter a safe-fail/degraded mode when an essential component fails to guarantee the safety of the system and all actors around.
