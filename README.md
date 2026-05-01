# KUKA LBR iiwa - Nullspace Reconfiguration for Obstacle Avoidance

![KUKA Avoidance](kuka_avoidance.gif)

This project implements a **Closed-Loop Inverse Kinematics (CLIK)** algorithm for a 7-DOF redundant manipulator (**KUKA LBR iiwa**). The robot is designed to reach a sequence of Cartesian waypoints while actively avoiding static or dynamic obstacles through **Null Space** reconfiguration.

The system integrates **MATLAB** and **CoppeliaSim** via the ZeroMQ Remote API.

## 🚀 Technical Features

- **Primary Task**: Cartesian waypoint tracking using PI control with velocity saturation.
- **Secondary Task (Null Space)**: 
    - **Khatib Potential Fields**: Repulsive forces to keep links at a safety distance.
    - **Vortex Fields**: Tangential components generated to bypass obstacles and avoid "face-to-face" local minima.
- **Task Relaxation**: Dynamic reduction of the primary task gain ($K_p$) when approaching critical obstacles to prioritize avoidance.
- **Stuck Detection**: An integral-based "anti-stall" mechanism that detects if the robot is stuck in a local minimum and provides an extra push to overcome the obstacle.
- **Damped Least Squares (DLS)**: Singularity management using a damped pseudo-inverse.
- **Telemetry**: Automatic generation of performance plots at the end of the simulation (Error, Minimum Distance, Joint Velocities).

## 📂 Repository Structure

```text
KUKA-iiwa-Obstacle-Avoidance/
├── src/                      # MATLAB source code
│   ├── main_avoidance.m      # Main simulation script
│   ├── controller_nullspace.m # CLIK controller function
│   ├── RemoteAPIClient.m     # CoppeliaSim ZMQ Client
│   └── cbor.m                # Communication helper
├── model/                    # Robot model
│   ├── kukanomesh.urdf       # URDF file for MATLAB
│   └── meshes/               # STL mesh files for the robot
├── sim/                      # Simulation files
│   └── kuka_scene.ttt        # CoppeliaSim (V-REP) scene
├── README.md                 # Documentation
└── LICENSE                   # MIT License


🛠️ Requirements
MATLAB (R2022b or later recommended)
Robotics System Toolbox
CoppeliaSim (v4.3 or later recommended)
📖 How to Run
Configuration: Ensure all files in the src folder are added to the MATLAB path.
CoppeliaSim:
Open the file sim/kuka_scene.ttt.
You do not need to press 'Play' manually; the MATLAB script will handle the simulation state.
MATLAB:
Open and run src/main_avoidance.m.
The robot will follow 4 waypoints (first, second, third, fourth) for 2 full laps.
Results: Once the mission is completed, the simulation stops automatically, and MATLAB generates telemetry plots.
📊 Results and Analysis
At the end of the mission, the system provides three fundamental plots for analysis:
Cartesian Error: Measures the accuracy of the end-effector relative to the targets.
Minimum Distance: Monitors the safety distance between any part of the robot and the obstacles.
Joint Velocities: Verifies the smoothness of the motion.
👥 Author
Giorgio De Santis (2026)
📄 License
Distributed under the MIT License. See LICENSE for more information.