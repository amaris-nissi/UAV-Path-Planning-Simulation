# UAV Path Planning Simulation

## Overview

This project simulates path planning for a UAV navigating through a dynamic environment. It implements and analyzes several path-planning algorithms, focusing on collision avoidance and efficient path determination in a simulated environment. Specifically, it features implementations of Depth-First Search (DPS), Dijkstra's algorithm, and the A* algorithm. Additionally, it incorporates a polynomial path-smoothing technique for more realistic UAV flight behavior.

## Algorithms Implemented

- **Depth-First Search (DFS):** Explores a graph data structure by traversing nodes as far as possible along each branch before backtracking.
- **Dijkstra's Algorithm:** Calculates the shortest path from a root node to all other nodes in a graph by creating a tree of shortest paths.
- **A*Algorithm:** Extends Dijkstra’s algorithm with a heuristic function estimating the total cost from a node to the goal, optimizing path selection.
- **Polynomial Smoothing:** Utilizes piecewise polynomials to smooth the trajectory of the UAV, minimizing derivatives of the polynomials over set position waypoints for smoother acceleration and velocity profiles.

## Project Structure

The project is divided into two versions: MATLAB and C++. Each version is contained in its directory with source code and accompanying files.

- **MATLAB Version:** Located in the `matlab/` directory, this version includes scripts for simulating the UAV's path planning using MATLAB's numerical computing environment.
- **C++ Version:** Found in the `cc/` directory, this version offers a more performance-optimized simulation, suitable for integration with real-world applications or high-fidelity simulations.

## Requirements

- **MATLAB:** MATLAB R2021a or newer.
- **C++ Compiler:** GCC 7.4.0 or newer, or an equivalent compiler with C++17 support.
- **Dependencies:** Eigen Library (for C++ version), MATLAB Robotics Toolbox.

## Usage

### MATLAB Version

1. Navigate to the `matlab/` directory.
2. Run `simulateQuadrotorEstimationAndControl.m` to simulate UAV dynamics and path following.

### C++ Version

1. Navigate to the `cc/` directory.
2. Compile the source files using your C++ compiler, e.g., `g++ -std=c++17 main.cc -o uav_planning`.
3. Run the compiled executable, e.g., `./uav_planning`.

## References

[1] Todd E. Humphreys, *Aerial Robotics Course Notes for ASE 479W,* The University of Texas at Austin, 2022.
[2] Tucker Haydon and Todd Humphreys, *Piecewise Polynomial Path Planning (P4).* Radionavigation Lab, 2020.
[3] Peter E. Hart, *A Formal Basis for the Heuristic Determination of Minimum Cost Paths,* IEEE Transactions of Systems Science and Cybernetics, No. 2, 1968.
