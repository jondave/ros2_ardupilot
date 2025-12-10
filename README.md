# ROS2 ArduPilot Integration

A ROS2 package for integrating ArduPilot-based autopilots (Pixhawk) with robotic systems. This package provides drivers and utilities to translate between Pixhawk flight controller outputs and ROS2 command velocity messages, supporting both manual RC control and autonomous waypoint navigation.

## Unitree Go2 Edu Quick Start
 - Turn on Go2.
 - Connect to the router on the backpack.
    - SSID - `Doug-Backpack`
    - Password – `Doug1234`
- SSH into the Go2's Jetson
```
ssh unitree@192.168.123.18
```
- Use this [repo](https://github.com/Unitree-Go2-Robot/go2_robot/tree/foxy-devel) for the Go2's ROS drivers, that converts `cmd_vel` to the ROS service to interface with the Go2's API.
- To run the drivers
```
source go2_ws/install/setup.bash
ros2 launch go2_driver go2_driver.launch.py
```

In a new terminal start the MAVROS script to interaface with the Pixhawk flight controller.
```
ros2 run mavros mavros_node --ros-args --param fcu_url:=serial:///dev/ttyACM1:115200 --param system_id:=255 --param component_id:=240 --param target_system_id:=1 --param target_component_id:=1
```

Next in a new terminal run the script to conver the MAVROS topics to `cmd_vel`.

```
cd sheep_ws/src/ros2_ardupilot/scripts
python3 omni_driver.py
```

### Manual Teleoperation using Pixhawk and the Radio Controller.
- Press the saftey `switch` button on the main GPS module, so the red light is solid.
- Arm / Disarm the Pixhawk - On the Radio controller flip switch `SF` (top left, back switch) from back to front, this `arms` and `disarms` the pixhawk. This can also be done in mission planner and using ROS
- Change Mode - On the Radio controller switch `SE` (top left front switch) changes the Pixhawk's mode, back is hold (the robot wont move), midde is manual (move using radio controller) and front is auto (autonomous movement.) 
- Back <-> Middle <-> Front --- Hold <-> Manual <-> Auto.

- Once all three have been done (safety switch, armed and manual mode) the robot can be teleoperated with the radio controller.
- Left stick up/down --- forward/backward (x vel)
- Left stick left/right --- crab left/right (y vel)
- RIght stick left/right --- turn left/right (z vel)

### Using Mission Planner to navigate to a waypoint
- In mission planner connect to Pixhawk.
- [Set waypoints using mission planner](https://ardupilot.org/copter/docs/common-planning-a-mission-with-waypoints-and-events.html).
- Change the Pixhawk's mode to `Auto` and once all the checks have been passed (safety switch and armed) the robot will move to the waypoints.

### Using ROS to navigate to a waypoint
- Press the safety `switch` on the GPS module and arm the Pixhawk.
- Change the Pixhawk mode to `Guided` either in mission planner or using ROS
```
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```
- In a SSH termianl run the command with you chosen latitude and longitude for the waypoint.
```
ros2 topic pub --once /mavros/setpoint_raw/global mavros_msgs/msg/GlobalPositionTarget "{D'}"
 header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
 coordinate_frame: 6,
 type_mask: 3068,
 latitude: 53.2679504,
 longitude: -0.5272549,
 altitude: 0.0,
 velocity: {x: 0.0, y: 0.0, z: 0.0},
 acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0},
 yaw: 0.0,
 yaw_rate: 0.0
}"
```

### MAVROS ROS Commands
- Arm / Disarm the Pixhawk
```
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: False}"
```

- Change the mode of the Pixhawk
- Hold
```
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'HOLD'}"
```
- Manual
```
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'MANUAL'}"
```
- Auto
```
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO'}"
```
- Guided
```
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```


## Overview

This package acts as a bridge between ArduPilot autopilots and ROS2-based robots (such as the Unitree Go2). It enables:

- **Manual Control**: Direct RC stick input translation to velocity commands
- **Autonomous Missions**: Waypoint navigation with ArduPilot mission planner
- **Hybrid Operation**: Seamless switching between manual and autonomous modes
- **System Diagnostics**: Connection verification and sensor debugging

## Features

- PWM-to-velocity mapping with configurable dead zones
- Multi-mode operation (MANUAL, AUTO, GUIDED, RTL)
- Support for omnidirectional (holonomic) and differential-drive robots
- Real-time diagnostics and status monitoring
- GPS-based waypoint publishing
- MAVROS middleware for Pixhawk communication

## Package Contents

### Scripts

| Script | Purpose |
|--------|---------|
| `omni_driver.py` | The main file to run. Hybrid driver for omnidirectional robots - switches between RC stick input (manual) and Pixhawk servo output (autonomous) |
| `pixhawk_mission.py` | Waypoint follower that translates Pixhawk throttle/steering servo outputs to velocity commands |
| `pixhawk_to_cmd_vel.py` | Rover-style translator for differential-drive robots using Servo 3 & 4 |
| `mission_commander.py` | Triggers mission start by arming the autopilot and switching to AUTO mode |
| `send_waypoint.py` | Publishes GPS waypoint targets for the Pixhawk to navigate |
| `pixhawk_debug.py` | Real-time debugging and visualization of PWM signals |
| `pixhawk_omni.py` | Specialized omni-directional vehicle driver |
| `test_connection.py` | Diagnostic tool to verify Pixhawk connection and RC channel functionality |

### Configuration

- `ardupilot/configs/` - ArduPilot parameter files and configuration templates

## Requirements

### Software
- ROS2 (tested on Humble)
- MAVROS package
- ArduPilot firmware on Pixhawk autopilot
- Python 3.8+

### Hardware
- Pixhawk autopilot (or compatible ArduPilot flight controller)
- USB connection to Pixhawk or telemetry link
- RC transmitter (for manual control)
- Robot platform (e.g., Unitree Go2)

## Installation

### 1. Install MAVROS
```bash
sudo apt install ros-<distro>-mavros ros-<distro>-mavros-extras
# Download geolocation database
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

### 2. Setup ArduPilot
- Upload ArduPilot firmware to your Pixhawk
- Configure parameters using Mission Planner or QGroundControl
- Set servo/channel functions appropriately

### 3. Build the Package
```bash
cd ~/sheep_ws
colcon build --packages-select ros2_ardupilot
source install/setup.bash
```

## Usage

### System Check - Verify Connection
```bash
ros2 run ros2_ardupilot test_connection
```

This will display:
- Pixhawk connection status
- RC input channels (stick positions)
- Servo output channels
- Armed/disarmed status
- Flight mode

### Manual Control (Omnidirectional)
```bash
ros2 run ros2_ardupilot omni_driver
```

**RC Stick Mapping** (Default):
- **Ch1 (Left Stick L/R)**: Forward/Backward
- **Ch2 (Right Stick L/R)**: Rotation
- **Ch4 (Left Stick U/D)**: Lateral (Crabbing)

The robot will subscribe to `/cmd_vel` and respond to your RC transmitter in MANUAL mode.

### Autonomous Mission Following
```bash
ros2 run ros2_ardupilot pixhawk_mission
```

Switch your Pixhawk to AUTO mode and upload a mission via Mission Planner. The script will:
1. Read Pixhawk servo outputs (steering and throttle)
2. Map PWM values to velocity commands
3. Publish to `/cmd_vel` for your robot to follow

**Servo Configuration**:
- Servo 1 (Channel 3): Throttle - controls forward/backward motion
- Servo 2 (Channel 4): Ground Steering - controls turn rate

### Send GPS Waypoint
```bash
ros2 run ros2_ardupilot send_waypoint
```

Edit the coordinates in `send_waypoint.py` to set your target location:
```python
self.target_lat = 53.2679504
self.target_lon = -0.5272549
```

### Start Mission
```bash
ros2 run ros2_ardupilot mission_commander
```

This will:
1. Wait for Pixhawk connection
2. Arm the system
3. Switch to AUTO mode to begin mission

### Differential-Drive Robot
For rovers with differential-drive steering (not omnidirectional):
```bash
ros2 run ros2_ardupilot pixhawk_to_cmd_vel
```

**Servo Mapping**:
- Servo 3 (Channel 2): Throttle
- Servo 4 (Channel 3): Ground Steering

## Configuration

### PWM Mapping Parameters

Edit the script's `__init__` method to adjust:

```python
# Speed limits
self.max_spd = 1.0      # Maximum linear velocity (m/s)
self.max_turn = 1.2     # Maximum angular velocity (rad/s)

# Channel indices
self.idx_man_fwd = 0    # Manual forward/back (Ch1)
self.idx_man_crab = 3   # Manual crabbing (Ch4)
self.idx_man_turn = 1   # Manual rotation (Ch2)

# Servo indices for autonomous mode
self.idx_auto_str = 1   # Steering servo (Servo 2)
self.idx_auto_thr = 2   # Throttle servo (Servo 3)
```

### Dead Zone Tuning

The `map_pwm()` function includes a dead zone to prevent stick drift:

```python
deadzone = 30  # PWM units (adjust for your radio)
```

Increase this value if you have unwanted drift, decrease if the robot feels unresponsive.

## Topics

### Subscribed Topics
- `/mavros/state` - Pixhawk connection and mode status
- `/mavros/rc/in` - RC input from transmitter
- `/mavros/rc/out` - Servo outputs from Pixhawk
- `/mavros/setpoint_raw/global` - Global GPS waypoints

### Published Topics
- `/cmd_vel` - Geometry Twist messages for robot velocity control

## Troubleshooting

### Pixhawk Not Connecting
1. Check USB cable connection
2. Verify MAVROS is installed: `ros2 pkg list | grep mavros`
3. Check device permissions: `ls -la /dev/ttyUSB*`
4. May need to add user to dialout group: `sudo usermod -aG dialout $USER`

### No RC Input
1. Ensure RC transmitter is powered and bound to receiver
2. Verify RC receiver is connected to Pixhawk RC IN port
3. Check channel calibration in Mission Planner
4. Use `pixhawk_debug.py` to visualize PWM values

### Robot Not Moving
1. Verify `/cmd_vel` topic is being published: `ros2 topic echo /cmd_vel`
2. Check if robot's motion controller is subscribed to `/cmd_vel`
3. Ensure Pixhawk is armed
4. Test connection with `test_connection.py`

### Drifting/Circling Issues
- Adjust `map_pwm()` center values to match your specific hardware
- Increase dead zone if stick is drifting
- Calibrate servos in Mission Planner for consistent centering

## Advanced

### Custom Servo Mapping
To use different servo channels:

1. Note your servo indices (0-based) in MAVROS
2. Update the index variables:
   ```python
   self.idx_throttle = 2  # Servo 3
   self.idx_steering = 3  # Servo 4
   ```
3. Verify with `test_connection.py` by watching servo values update

### Velocity Scaling
Adjust maximum velocities for your robot:
```python
self.max_spd = 0.5      # Slower linear velocity
self.max_turn = 0.8     # More gradual turns
```

### PWM Range Calibration
If your radio or servos have non-standard PWM ranges:
```python
# Default: 1000-2000 with 1500 center
# Adjust to your hardware:
vel_x = self.map_pwm(pwm_value, min_pwm, max_pwm, center_pwm, out_min, out_max)
```

## Authors

Developed for Unitree Go2 integration with ArduPilot autopilots.

## License

[Specify your license here, e.g., MIT, Apache 2.0]

## Support

For issues and questions:
1. Check the Troubleshooting section
2. Review MAVROS documentation: http://wiki.ros.org/mavros
3. Reference ArduPilot Rover documentation: https://ardupilot.org/rover/

## References

- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [ArduPilot Rover Documentation](https://ardupilot.org/rover/)
- [ROS2 Geometry Twist](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)
- [Pixhawk Flight Controller](https://pixhawk.org/)
