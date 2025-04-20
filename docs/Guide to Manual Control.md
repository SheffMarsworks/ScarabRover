# Guide to Manual Control: Matek H743-WING V3 with ArduRover and ROS2

This guide covers the complete setup process for controlling a skid-steering rover using the Matek H743-WING V3 flight controller with ArduRover firmware and ROS2/MAVROS integration.

## Flight Controller Setup

### Flashing ArduRover Firmware

1. Download the latest ArduRover firmware for the Matek H743-WING V3 from the [ArduPilot firmware page](https://firmware.ardupilot.org/)
2. Flash the firmware using STM32CubeProgrammer:
   - Connect the FC via USB while holding the boot button
   - Select the downloaded firmware file
   - Flash the controller

### Initial Configuration

1. Install QGroundControl on your computer
2. Connect the flight controller via USB
3. QGroundControl should automatically detect the board
4. Perform sensor calibration:
   - Accelerometer calibration
   - Set frame type to "Rover"

## ArduRover Configuration for Skid Steering

Configure the following parameters for skid steering:

```
SERVO1_FUNCTION = 73  # ThrottleLeft
SERVO3_FUNCTION = 74  # ThrottleRight
```

Set the motor PWM type:
- For standard RC ESCs: `MOT_PWM_TYPE = 0` (Normal)

### Motor Connection and Testing

1. Connect your motor ESCs to the flight controller:
   - Left motor/ESC to SERVO1 output
   - Right motor/ESC to SERVO3 output
2. In QGroundControl or Mission Planner:
   - Navigate to the Motor Test section
   - Test each motor individually to verify proper connection and direction
   - Adjust motor direction in configuration or by swapping motor wires if needed
3. If motors don't spin at low throttle, adjust minimum PWM values

## Controlling rover with MAVROS using ROS2 

### Prerequisites

1. Install ROS2 Humble
2. Install MAVROS packages:
   ```bash
   sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
   ```
3. Install joystick packages:
   ```bash
   sudo apt install ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-teleop-twist-joy
   ```

Before launching, ensure you have proper permissions for the serial port:

```bash
sudo chmod 666 /dev/ttyACM0
```

For a permanent solution, create a udev rule:

```bash
echo 'KERNEL=="ttyACM0", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ardupilot.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Controlling the Rover with ROS2

### Launch the System

Start the ROS2 nodes:

```bash
ros2 launch rover_bringup real_robot.launch.py
```

### Set GUIDED Mode and Arm

Before sending velocity commands, set the vehicle to GUIDED mode:

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```

Arm the rover (note the value should be "true" to arm):

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

To disarm when finished:

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

### Joystick Control

Connect your joystick before launching the system. Once launched:

1. Ensure the rover is in GUIDED mode
2. Make sure the rover is armed
3. Press the enable button on your joystick (R1)
4. Use the left stick to control linear movement:
   - Y-axis: Forward/backward motion
   Use the right stick to control angular movement:
   - X-axis: Left/right turning

The teleop node publishes velocity commands to `/mavros/setpoint_velocity/cmd_vel_unstamped`, which ArduPilot uses to generate appropriate PWM signals for the motors.

## Troubleshooting

### Common Issues

1. **No communication with flight controller**:
   - Check USB connection
   - Verify correct port and baud rate in the launch file `mavros_controller.launch.py`
   - Ensure proper permissions with `sudo chmod 666 /dev/ttyACM0`

2. **Motors don't respond**:
   - Verify you're in GUIDED mode: `ros2 topic echo /mavros/state`
   - Check arming status
   - Verify motor connections and PWM outputs
   - Test motors directly in QGroundControl

## Additional Resources

- [ArduPilot Rover Documentation](https://ardupilot.org/rover/)
- [Matek H743-WING Documentation](https://ardupilot.org/rover/docs/common-matekh743-wing.html)
- [MAVROS GitHub Repository](https://github.com/mavlink/mavros)
- [ROS2 Joystick Control Documentation](https://index.ros.org/p/teleop_twist_joy/)
