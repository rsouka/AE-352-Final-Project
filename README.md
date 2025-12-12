# AE 352 Final Project: 6-DOF Quadcopter Dynamical Model & Simulation

## Overview
This repository contains the implementation of a full dynamical model for a symmetric four-axis quadcopter. The project focuses on deriving and simulating the nonlinear equations of motion to create an accurate "digital twin" of a physical drone. Quadcopters maneuver by varying motor speeds, which directly determine the forces and torques acting on the airframe. This project captures both the translational motion of the center of mass and the rigid-body rotational dynamics that define attitude, enabling precise prediction of flight behavior under various control inputs.

## Key Features
- Full 6-DOF Representation: Models position ($x, y, z$) and attitude (roll, pitch, yaw) coupled with external forces.
- Physics-Based Drive: Motion is driven solely by physical motor-induced rotor speeds.
- Realistic Constraints:
  - Feasible power usage consistent with motor, rotor, and battery limits.
  - Physically implementable parameters using off-the-shelf components.
  - Total mass constraints between 0.1 kg and 10 kg.
- Validation Scenarios: Verified against specific flight maneuvers including hovering, circular tracking, and compound trajectories.

## Verification & Performance Goals
The model has been validated against three distinct performance benchmarks to ensure stability and accuracy:
- Static Hover: Maintain a stable hover at 1 meter altitude for 2 minutes
- Circular Trajectory: Track a circle of 2m radius at 1m altitude with a velocity of 0.5 m/s.
- Compound Maneuver: Complete a full mission profile: Takeoff $\rightarrow$ Translation $\rightarrow$ Hover $\rightarrow$ Turn $\rightarrow$ Translation $\rightarrow$ Hover $\rightarrow$ Controlled Landing
