# SO-101 ROS 2 Workspace

ROS 2 workspace for the [SO-101 arm](https://github.com/TheRobotStudio/SO-ARM100)
with:

- Gazebo simulation
- MoveIt 2 planning and RViz
- Closed-loop real-robot control for Feetech STS3215 servos
- A Waveshare ESP32 USB-to-servo-bus bridge
- Optional task-server and Alexa interface

The workspace supports both simulation and the physical arm. The real-robot
driver reads servo positions every control cycle and starts commands from the
measured pose.

## Current status

| Capability | ROS 2 Humble |
|---|---|
| Real arm read-only launch | Tested |
| Real arm closed-loop control | Tested |
| MoveIt execution on real arm | Tested from a non-folded pose |
| Robot model spawning in Gazebo | Tested |
| Gazebo controller startup and motion | Needs verification on the target machine |
| ROS 2 Jazzy | Not tested |

Use ROS 2 Humble on Ubuntu 22.04 for the known-tested real-robot setup.

## Repository layout

```text
firmware/so101_bridge/so101_bridge.ino   ESP32 USB-to-servo-bus bridge
tools/servo_check.py                     Servo bus and calibration diagnostic
src/so101_description/                   URDF/xacro, meshes, Gazebo, RViz
src/so101_controller/                    ros2_control hardware plugin
src/so101_moveit/                        MoveIt configuration
src/so101_msgs/                          ROS action definitions
src/so101_remote_control/                Task server and Alexa interface
src/so101_bringup/                       Real-robot and simulation launch files
src/moveit_api/                          MoveIt helper library
```

## How the system works

```text
RViz / task server
        |
        v
MoveIt 2 -> FollowJointTrajectory
        |
        v
arm_controller / gripper_controller
        |
        v
So101Interface (real robot) or gz_ros2_control (simulation)
        |
        v
USB serial -> ESP32 bridge -> 6x Feetech STS3215 servos
```

For the real arm, `So101Interface` performs position feedback reads and
position commands. The servo firmware still performs the servo-level position
control; this project does not add a ROS-side PID loop.

## Hardware

- SO-101 arm with six Feetech STS3215 servos.
- Waveshare ESP32 servo driver board.
- Separate servo power supply. USB does not power the servos.
- Ubuntu 22.04 with ROS 2 Humble.
- A LeRobot calibration JSON for the arm.

Servo IDs are expected to be:

| ID | Joint |
|---:|---|
| 1 | `shoulder_pan` |
| 2 | `shoulder_lift` |
| 3 | `elbow_flex` |
| 4 | `wrist_flex` |
| 5 | `wrist_roll` |
| 6 | `gripper` |

## Install ROS 2 Humble dependencies

Install ROS 2 Humble first using the
[official instructions](https://docs.ros.org/en/humble/Installation.html).
Then install the project dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-xacro \
  ros-humble-ros-gz \
  ros-humble-ign-ros2-control \
  libserial-dev \
  python3-serial \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-flask
```

Give the current user access to USB serial devices:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership. If Ubuntu's `brltty`
claims the adapter, remove it:

```bash
sudo apt remove brltty
```

## Clone and build

The repository itself is the workspace root:

```bash
git clone https://github.com/Combatant1/so101_Ros2_gazebo_actualrobot.git ~/so101_ws
cd ~/so101_ws

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For every new shell:

```bash
cd ~/so101_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Flash the ESP32 bridge

1. Install the Espressif ESP32 board package in Arduino IDE.
2. Select **ESP32 Dev Module**.
3. Open `firmware/so101_bridge/so101_bridge.ino`.
4. Verify `S_RXD` and `S_TXD` against the documentation for the exact board
   revision. The defaults are GPIO 18 and GPIO 19.
5. Upload the firmware. Hold the BOOT button if the board remains at
   `Connecting...`.

The firmware uses:

```text
PC <-> ESP32 USB serial: 115200 baud
ESP32 <-> servo bus:     1000000 baud
```

`USB_BAUD` in the firmware must match `baud_rate` in
`src/so101_description/urdf/so101_ros2_control.xacro`.

## Calibration

Calibrate the arm with LeRobot before launching ROS:

```bash
lerobot-calibrate
```

The calibration JSON is normally under:

```text
~/.cache/huggingface/lerobot/calibration/robots/so101_follower/
```

The driver uses the calibrated servo tick ranges. It does not apply the
LeRobot homing offsets a second time.

Calibration parameters are stored in:

```text
src/so101_description/urdf/so101_ros2_control.xacro
```

The relevant convention is:

- Arm joint `zero_tick`: midpoint of `range_min` and `range_max`
- Gripper `zero_tick`: `range_min`
- `min_tick` and `max_tick`: recorded calibrated range
- `direction`: `1` or `-1`

The current checked-in values are:

| Joint | ID | `zero_tick` | `min_tick` | `max_tick` |
|---|---:|---:|---:|---:|
| `shoulder_pan` | 1 | 2133.5 | 792 | 3475 |
| `shoulder_lift` | 2 | 2085 | 909 | 3261 |
| `elbow_flex` | 3 | 1921 | 801 | 3041 |
| `wrist_flex` | 4 | 2009.5 | 834 | 3185 |
| `wrist_roll` | 5 | 2047.5 | 0 | 4095 |
| `gripper` | 6 | 2039 | 2039 | 3534 |

Treat these values as a starting point and verify them with the read-only
procedure below.

## Servo bus check

Only one application may use the serial port at a time. Stop the diagnostic
tool before launching ROS.

```bash
cd ~/so101_ws
python3 tools/servo_check.py \
  /dev/ttyUSB0 \
  --calib ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<id>.json \
  --watch
```

Expected results:

- All six servo IDs answer.
- All servos report operating mode `0` (position mode).
- Joint angles change smoothly when the arm is moved by hand.

To release servo torque:

```bash
python3 tools/servo_check.py /dev/ttyUSB0 --torque-off
```

If the adapter echoes transmitted bytes, retry with `--echo`.

## Launch the real arm

### Read-only mode

Read-only mode never enables torque and never sends commands:

```bash
cd ~/so101_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch so101_bringup actual_robot.launch.py \
  port:=/dev/ttyUSB0 \
  read_only:=true
```

Move the arm by hand and verify that RViz follows the physical pose. Check
joint direction and zero alignment before enabling torque.

### Closed-loop control

After stopping the read-only launch with `Ctrl-C` and confirming that the port
is free:

```bash
fuser -v /dev/ttyUSB0
ros2 launch so101_bringup actual_robot.launch.py
```

Keep a hand near the servo power switch during first bring-up. Start with a
non-folded pose and use low velocity scaling in RViz.

To use a different serial device:

```bash
ros2 launch so101_bringup actual_robot.launch.py \
  port:=/dev/ttyACM0
```

### Optional remote interface

The bringup launch includes the restored `so101_remote_control` package.
It starts the task server and Alexa Flask interface. The Flask server uses
port `5000`.

To launch only the remote interface:

```bash
ros2 launch so101_remote_control remote_interface.launch.py
```

## Launch simulation

```bash
cd ~/so101_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch so101_bringup sim_robot.launch.py
```

RViz-only model display:

```bash
ros2 launch so101_description display.launch.py
```

The robot model has been verified to spawn in Gazebo. If the controller
spawners wait indefinitely for `/controller_manager/list_controllers`, inspect
the Gazebo and `ign_ros2_control` plugin output before attempting MoveIt
motion.

## Rest pose and MoveIt collision

The fully folded rest pose can be reported as a self-collision by MoveIt
because the detailed collision meshes contact each other. Start from a
non-folded pose when planning.

If you need to move away from the folded pose directly, use a slow trajectory
and keep clear of the gripper:

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [wrist_roll, wrist_flex, elbow_flex, shoulder_lift, shoulder_pan],
    points: [{positions: [0.0, 1.07, 1.42, -1.47, 0.05],
              time_from_start: {sec: 6}}]}"
```

## Configuration reference

### Hardware parameters

Configured in `so101_ros2_control.xacro`:

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | USB serial device |
| `baud_rate` | `115200` | PC-to-ESP32 USB baud rate |
| `read_only` | `false` | Read feedback without torque or commands |
| `startup_delay` | `2.0` | ESP32 reset delay after opening serial |
| `timeout_ms` | `20` | Servo reply timeout |
| `read_health` | `false` | Read voltage and temperature |
| `acceleration` | `254` | Servo acceleration |
| `disable_torque_on_deactivate` | `false` | Keep torque when ROS stops |
| `bus_echo` | `false` | Discard echoed USB bytes when enabled |
| `max_consecutive_failures` | `20` | Communication failures before error |

### Controllers

Configured in `src/so101_controller/config/so101_controllers.yaml`:

- Update rate: `50 Hz`
- Position command interfaces
- Closed-loop trajectory execution
- Arm goal tolerance: `0.1 rad`
- Gripper goal check disabled for object grasping

MoveIt execution settings are in:

```text
src/so101_moveit/config/moveit_controllers.yaml
```

## Troubleshooting

### Serial port cannot be opened

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
fuser -v /dev/ttyUSB0
groups
```

Typical causes are another process using the port, a changed device name, or
missing `dialout` membership.

### Servo does not answer

Check servo power, bus wiring, ESP32 pins, firmware, and baud rates. Then run:

```bash
python3 tools/servo_check.py /dev/ttyUSB0
```

### A joint moves in the wrong direction

Change that joint's `direction` from `1` to `-1` in
`so101_ros2_control.xacro`, rebuild, and relaunch.

### A pose is offset

Adjust that joint's `zero_tick`. One degree is approximately 11.4 ticks.

### MoveIt reports a collision at the start

Start from a non-folded pose. See
[Rest pose and MoveIt collision](#rest-pose-and-moveit-collision).

### ROS 2 CLI reports `!rclpy.ok()`

Restart the ROS 2 daemon:

```bash
ros2 daemon stop
ros2 daemon start
```

### Gazebo controller manager is unavailable

Check that the simulation launch is using the correct Humble control plugin:

```bash
ros2 pkg prefix ign_ros2_control
ros2 pkg prefix gz_ros2_control
ros2 service list | grep controller_manager
```

The robot can spawn successfully even when the control plugin or controller
manager fails, so always inspect the Gazebo process output.

## Development checks

Build the core packages:

```bash
cd ~/so101_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to \
  so101_bringup \
  so101_controller \
  so101_description \
  so101_moveit
```

Check the working tree:

```bash
git status --short --branch
git diff --check
```

## Roadmap

- Verify and harden Gazebo controller startup on clean Humble and Jazzy installs.
- Test the stack on ROS 2 Jazzy.
- Add CI builds for Humble and Jazzy.
- Add stable udev naming for the USB adapter.
- Measure real cycle timing, settling error, and servo temperature.
- Improve collision meshes so planning can start from the folded pose.
- Add diagnostics for servo speed, load, voltage, and temperature.
- Add startup and park trajectories.
- Improve task-server goal error handling and reteach stored poses.
- Add emergency-stop and command watchdog support.

## Credits and license

- Robot model: [TheRobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
- Calibration conventions: [Hugging Face LeRobot](https://github.com/huggingface/lerobot)
- Built with ROS 2, `ros2_control`, MoveIt 2, Gazebo, and
  [LibSerial](https://github.com/crayzeewulf/libserial)
- Packages declare the Apache-2.0 license and retain their package license
  files.
