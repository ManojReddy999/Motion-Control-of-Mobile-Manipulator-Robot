# MAE204 - Final Project: Mobile Manipulator (YouBot) Motion Control

This repository contains the MATLAB code and report for the MAE204 Final Project, focusing on the motion control of a KUKA YouBot mobile manipulator robot.  The project implements a complete control system for a pick-and-place task, including kinematics simulation, trajectory generation, and feedback control.  The simulation is visualized using CoppeliaSim.

**Author:** Manoj Kumar Reddy Manchala

## Project Overview

The goal of this project is to design and implement a control system that enables the YouBot to autonomously:

1.  Navigate to a specified location.
2.  Pick up a cube.
3.  Place the cube at a target destination.

[![Simulation Preview](Videos/Best_Case.gif)
[![Simulation Video](https://img.youtube.com/vi/fprW7P0tV-8/0.jpg)](https://youtu.be/fprW7P0tV-8?si=zktSQMS4hKUYIwnB)


This is achieved through the following key components:

*   **Kinematics Simulator (`NextState.m`):**  Predicts the robot's next state (position and orientation) based on its current state, control inputs (joint and wheel velocities), and a time step.  It uses the Euler method for integration.  This simulates the robot's motion.
*   **Reference Trajectory Generator (`Trajectory_Generator.m`):**  Generates a smooth, eight-segment trajectory for the robot's end-effector to follow during the pick-and-place operation. This trajectory specifies the desired position, orientation, and gripper state over time.  It takes into account initial and final positions of the cube, and standoff positions.
*   **Feedback Controller (`FeedbackControl.m`):** Implements a feedforward plus proportional-integral (PI) control law to minimize the error between the robot's actual trajectory and the reference trajectory.  It calculates the necessary joint and wheel velocities to correct deviations.  It utilizes the `pinv` function with a tolerance to avoid singularities in the Jacobian calculation.
*   **Main Script (`Final_Project.m`, and others):**  A wrapper script that ties together the simulator, trajectory generator, and controller.  It runs the simulation, generates output data, and creates a CSV file for visualization in CoppeliaSim.  Separate scripts are provided for different test cases.
*    **Vector to Transformation Matrix Converter (`VectorToTrans_form.m`):** It converts the vector to homogenous transformation matrix

## Files Included

The following MATLAB `.m` files are included:

*   `best_case.m`:  Main script for the "best case" scenario (well-tuned PI controller).
*   `FeedbackControl.m`:  Implements the feedforward + PI feedback controller.
*   `Final_Project.m`:  Main wrapper script (likely used as a base for other scenario scripts).
*   `high_oscillations.m`: Main script for the "high oscillations" scenario.
*   `new_task.m`:  Main script for the "new task" scenario (different cube placement).
*   `NextState.m`:  Kinematics simulator.
*   `overshoot.m`:  Main script for the "overshoot" scenario (poorly tuned PI controller).
*   `resetCumError.m`: Resets the integral error.
*   `slow_convergence.m`:  Main script for the "slow convergence" scenario.
*   `Test_Trajector yGen.m`: A script likely used for testing the trajectory generator independently.
*   `Trajectory_Generator.m`:  Generates the reference trajectory.
*   `VectorToTrans_form.m`: Converts a 12-element vector to a 4x4 homogeneous transformation matrix.

## Dependencies

*   **MATLAB:** The code is written in MATLAB.
*   **Modern Robotics Toolbox:** This project utilizes functions from the Modern Robotics library (accompanying the textbook *Modern Robotics: Mechanics, Planning, and Control*). You will need to have this library installed and added to your MATLAB path.  (http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
*   **CoppeliaSim:**  This project uses CoppeliaSim for visualization. You need to download and install CoppeliaSim.  Specifically, you'll need scenes 6 and 8 from the CoppeliaSim introduction page: (http://hades.mech.northwestern.edu/index.php/CoppeliaSim_Introduction)

## Running the Code

1.  **Install Dependencies:** Ensure you have MATLAB, the Modern Robotics Toolbox, and CoppeliaSim installed.

2.  **Set up CoppeliaSim:**  Download CoppeliaSim scenes 6 and 8 and place them in a known directory.

3.  **Open MATLAB:** Open MATLAB and navigate to the directory containing the project files.

4.  **Run a Scenario:**  Execute one of the main scenario scripts (e.g., `best_case.m`, `overshoot.m`, `new_task.m`, `slow_convergence.m`, `high_oscillations.m`).  Each script will:
    *   Generate a `.csv` file (e.g., `best_case.csv`) containing the robot's trajectory.
    *   Create a `.mat` file containing error data.

5.  **Visualize in CoppeliaSim:**
    *   Open CoppeliaSim and load Scene 6 (for the main simulation) or Scene 8 (for testing `Trajectory_Generator.m` using `Test_Trajector yGen.m`).
    *   Use the "Import CSV" functionality in CoppeliaSim to load the generated `.csv` file.
    *   Run the simulation in CoppeliaSim to visualize the robot's motion.

6.  **Analyze Results:**  The `.mat` file contains error data (the `Xerr` variable), which can be plotted in MATLAB to analyze the controller's performance.  Each main script should include code to generate this plot.

## Detailed Explanation of Each Script

*   **`best_case.m`**:  This script runs the simulation with well-tuned Kp and Ki values (1.5\*Identity and 0.001\*Identity, respectively) to achieve good tracking performance with minimal overshoot.

*   **`overshoot.m`**: This script uses higher Ki and Kp values (2.85\*Identity and 2.37\*Identity) to demonstrate overshoot.  The integral term is more aggressive, leading to oscillations.

*   **`new_task.m`**:  This script modifies the initial and final positions of the cube ([0.5, 0] and [1, -0.5], respectively) while keeping the orientations the same as the best case.  The same Kp and Ki as the best case are used.

*   **`slow_convergence.m`**:  This script uses very low Kp and Ki values (0.2\*Identity and 0\*Identity) to demonstrate slow convergence.  The robot does not reach the cube in time to pick it up.

*   **`high_oscillations.m`**:  This script uses a moderate Kp (0.75\*Identity) and a high Ki (10\*Identity) to induce significant oscillations.

*   **`Test_TrajectoryGen.m`:** This script tests only the TrajectoryGenerator.m.

*   **`Final_Project.m`:** It is likely that it can be used as a base to create customized cases.

This README provides a comprehensive guide to understanding, running, and analyzing the code for this project.  Remember to install the necessary dependencies and follow the steps carefully to reproduce the results.
