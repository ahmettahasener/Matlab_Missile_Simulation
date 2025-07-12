# Matlab Missile Simulation
This project is an interactive missile flight simulation built with MATLAB App Designer. It provides a visual and analytical tool for understanding missile dynamics by allowing users to define various launch and flight parameters. The application demonstrates core engineering principles, specifically how physical forces influence a missile's trajectory.
# Project Overview
The main purpose of this application is to offer a straightforward platform for simulating ballistic trajectories under realistic atmospheric conditions. Users can manipulate key design and operational parameters to see the interplay between thrust, gravity, and aerodynamic drag. This makes it a valuable educational tool for those interested in aerospace dynamics and a practical example of MATLAB's interactive application development capabilities. The simulation uses Newton's laws of motion to predict the missile's path, velocity, and acceleration by integrating forces over small time steps.
# Key Features
-Customizable Launch Parameters: Define initial position, velocity, elevation, and azimuth angles, along with missile mass (initial and dry), thrust, burn time, fuel flow rate, reference area, and drag coefficient.

-Realistic Physics Model: Simulates missile movement based on Newton's Second Law. It dynamically calculates aerodynamic drag using a simplified standard atmosphere model for air density, applying a constant gravitational acceleration.

-Dynamic 3D Visualization: Offers a clear 3D plot of the missile's flight path, providing an intuitive understanding of trajectory changes.

-Detailed Performance Graphs: Presents time-series plots for total velocity and its components (Vx, Vy, Vz), total acceleration and its components (Ax, Ay, Az), and aerodynamic drag force.

-Tabulated Data Output: Displays numerical data in tables, including time, position, velocity, acceleration, and drag force for detailed analysis.

-Targeting and Outcome Analysis: Allows users to define a 3D target. The application then determines if the missile successfully hit the target (within tolerance) and shows the final distance.

-Interactive Simulation Summary: Provides a concise overview of the simulation run, including input parameters, flight duration, maximum altitude, and target engagement status.
# Screenshots
- Target Hit Scenario
<img width="1987" height="1441" alt="Success" src="https://github.com/user-attachments/assets/bab6fbe4-73c0-4988-812f-2a501a5c004a" />


- Target Miss Scenario
<img width="1987" height="1447" alt="Fail" src="https://github.com/user-attachments/assets/447e3a3e-d20f-4a79-b12a-32e34dba50eb" />
