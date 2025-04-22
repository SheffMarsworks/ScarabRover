# Project Marsworks: Scarab Mars Rover

ROS2 workspace developed to achieve manual control and autonomous navigation in both simulation and real-world environments for the [Anatolian Rover Challenge (ARC) 2025](https://www.anatolianrover.space/arc-25-missions).

[![Static Badge](https://img.shields.io/badge/Renzo%20Damian-orange?label=Software%20Lead&link=https%3A%2F%2Fgithub.com%2Frenzodamgo)
](https://github.com/renzodamgo)

Main components:

- Rover Simulation (Gazebo)
- Rover Teleoperation (MAVROS)
- Rover Autonomous Navigation (RTAB + NAV2)

<center> <img src="assets/rover_gazebo_depth.gif" alt="Scarab Rover Simulation in Gazebo"> </center>

## Prerequisites

- ROS2 (Humble or later recommended)
- Ubuntu 22.04 Jammy Jellyfish
- Gazebo Ignition

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/SheffMarsworks/ScarabRover.git

   cd ScarabRover
   ```

2. Install dependencies:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   sudo apt install ros-humble-joy-teleop
   sudo apt install ros-humble-joint-state-broadcaster
   sudo apt install ros-humble-diff-drive-controller
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
   ```

3. Build the workspace:

   ```bash
   colcon build
   ```

## Documentation

- [Run Simulation (Gazebo)](https://github.com/SheffMarsworks/ScarabRover/blob/main/docs/Guide%20to%20Run%20Simulation.md): Load URDF and World, test Rover with joystick or keyboard in simulation.
- [Guide to Manual Control](https://github.com/SheffMarsworks/ScarabRover/blob/main/docs/Guide%20to%20Manual%20Control.md): Connect flight controller to GCS using MAVROS, and control real rover with joystick.

## Acknowledgments

- [Project Marsworks Software Team](https://marsworks.sites.sheffield.ac.uk/team#h.4kuzgvqqcu52)
- [@Jan](https://github.com/JanUniAccount) for providing the [original Mars rover simulation URDF](https://github.com/JanUniAccount/mars_rover_pkg) in ROS1

### Contributors
<a href="https://github.com/SheffMarsworks/ScarabRover/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=SheffMarsworks/ScarabRover" />
</a>
