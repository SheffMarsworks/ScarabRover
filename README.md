# Marsworks Scarab

This repository contains the ROS2 workspace for simulating the Marsworks Scarab rover using Gazebo.

<center> <img src="assets/rover_gazebo_depth.gif" alt="Scarab Rover Simulation in Gazebo"> </center>

## Prerequisites

- ROS2 (Humble or later recommended)
- Gazebo Ignition
- Ubuntu 22.04 or later

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/renzodamgo/ScarabRover.git
   ```

2. Navigate to the workspace:

   ```bash
   cd ScarabRover
   ```

3. Install dependencies:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   sudo apt install ros-humble-joy-teleop
   sudo apt install ros-humble-joint-state-broadcaster
   sudo apt install ros-humble-diff-drive-controller
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
   ```

4. Build the workspace:
   ```bash
   colcon build
   ```

## Run Simulation

To run the Scarab rover simulation, follow these steps:

1. **Source the Workspace**
   First, ensure your environment is set up by sourcing the workspace:
   ```bash
   source install/setup.bash
   ```

2. **Launch the Simulation**
   **Important:** If you plan to control the rover with a joystick, connect it before running this command.
   ```bash
   ros2 launch rover_bringup simulated_robot.launch.py
   ```

3. **Control the Robot**
   To move the robot using a joystick:
   - **Speed Control:** Move the L3 stick from top to bottom to increase or decrease speed.
   - **Direction Control:** Use the R3 stick from left to right to change direction.
   - **Deadman Button:** Press the deadman button (R1 or RB) to enable command reception.

   <center>
   <img src="assets/scarab-joystick.gif" alt="Scarab Rover Joystick Simulation in Gazebo">
   </center>

## Project Structure

```
└── src
    ├── rover_bringup
    │   ├── launch
    │   │   ├── real_robot.launch.py
    │   │   └── simulated_robot.launch.py
    │   └── ...
    └── rover_description
        ├── launch
        │   ├── display.launch.py
        │   └── gazebo.launch.py
        ├── meshes
        │   ├── base_link.stl
        │   ├── suspension_left_link.stl
        │   └── suspension_right_link.stl
        ├── rviz
        │   └── display.rviz
        ├── urdf
        │   ├── materials.xacro
        │   └── robot.urdf
        └── worlds
            ├── empty.sdf
            └── wro_world.sdf
```

- `rover_bringup`: Contains launch files for both simulated and real robot scenarios.
- `rover_description`: Includes URDF files, mesh files, and world descriptions for Gazebo simulation.

## Acknowledgments

- Marsworks team
- ROS2 community
- [Jan](https://github.com/JanUniAccount) for providing the [original Mars rover simulation](https://github.com/JanUniAccount/mars_rover_pkg) in ROS1, which served as the foundation for this ROS2 implementation
