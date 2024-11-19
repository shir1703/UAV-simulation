Introduction
This project implements a simulation for autonomous unmanned aerial vehicles (UAVs). Each UAV flies to specified target points and circles around them when reached. The simulation dynamically processes UAV movements, adhering to physical constraints like minimum turn radius and constant speed.

Project Structure
Key Files
main.cpp

Entry point of the simulation.
Defines UAV behavior, movement logic, and simulation flow.
UAV.h

Header file for the UAV class.
Contains method declarations and member variables for UAV properties and behavior.
UAV.cpp

Implementation of the UAV class.
Includes logic for linear and circular motion, azimuth calculations, and state tracking.
Input Files

SimParams.ini: Initializes simulation parameters such as the number of UAVs, time step, initial conditions, and simulation duration.
SimCmds.txt: Contains flight commands for UAVs, including the target coordinates and execution times.
Output Files

UAV<n>.txt: For each UAV, logs its state (time, x, y, azimuth) at every simulation step.

How to Run the Simulation
Prerequisites
Visual Studio (or any C++11-compatible compiler).
Basic understanding of simulation concepts and C++.
Steps
Clone or download the repository.
Open the provided Visual Studio solution file (<solution_name>.sln).
Verify the SimParams.ini and SimCmds.txt files are in the working directory and properly configured.
Build the project in Visual Studio.
Run the simulation executable.
Check the output files (UAV<n>.txt) for results.
