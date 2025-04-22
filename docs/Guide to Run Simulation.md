# Guide to run simulation (Gazebo Ignition)

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
   # Launch with joystick control
   ros2 launch rover_bringup simulated_robot.launch.py teleop:=joystick
   
   # Launch with keyboard control
   ros2 launch rover_bringup simulated_robot.launch.py teleop:=keyboard
   
   # Launch without teleop controls (for autonomous operation or testing)
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
