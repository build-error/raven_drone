# Raven Drone

![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros&logoColor=white)
![PX4](https://img.shields.io/badge/PX4-v1.17.0-orange)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)
![Status](https://img.shields.io/badge/Status-Active%20Development-brightgreen)
![QGroundControl](https://img.shields.io/badge/QGroundControl-Compatible-blue)
![RViz2](https://img.shields.io/badge/RViz2-Supported-green)
![SITL](https://img.shields.io/badge/PX4-SITL-orange)
![Platform](https://img.shields.io/badge/Platform-Holybro%20X650-lightgrey)
![Pixhawk 6X](https://img.shields.io/badge/Pixhawk-6X-blue)
![Raspberry Pi 5](https://img.shields.io/badge/Raspberry%20Pi-5-C51A4A?logo=raspberrypi&logoColor=white)


Raven Drone is a research and development repository for autonomous drone control using **PX4**, **ROS 2 Jazzy**, and **Computer Vision**.

The project serves as a testbed for developing custom PX4 flight modes, velocity-based control, autonomous navigation, visualization tools, and future **Image-Based Visual Servoing (IBVS)** algorithms.

<!-- The long-term objective of this project is to develop a complete vision-based autonomous drone system capable of target tracking and visual servoing using onboard or offboard perception and control. -->

## Current Features

* Custom PX4 External Flight Modes
* Keyboard Teleoperation
* Body-Frame Velocity Control
* Body-to-NED Velocity Transformation
* Autonomous Square Trajectory Tracking
* RViz-Based Visualization Tools
* PX4 ROS 2 Interface Library Integration
* PX4 SITL Simulation Support

## Future Goals

* ArUco-Based Target Tracking
* Image-Based Visual Servoing (IBVS)
* Visual Target Following
* Autonomous Landing
* Real-World Deployment on Holybro X650

## Repository Structure

```text
raven_drone
├── drone_ws
├── Micro-XRCE-DDS-Agent
├── PX4-Autopilot
└── README.md
```

### PX4-Autopilot

PX4 flight stack used for Software-In-The-Loop (SITL) simulation and future deployment on real hardware.

### Micro-XRCE-DDS-Agent

Micro XRCE-DDS Agent used to bridge communication between PX4 and ROS 2.

### drone_ws

ROS 2 Jazzy workspace containing custom packages, flight modes, and visualization tools developed as part of this project.

### README.md

Project documentation, setup instructions, and development roadmap.

## ROS 2 Packages

The ROS 2 workspace contains several custom packages developed for autonomous drone control, simulation, and visualization.

### teleop_mode
[PX4 ROS2 Teleop custom mode demo](https://youtu.be/iKkh6K9avOA)\
\
A custom PX4 external flight mode built using the PX4 ROS 2 Interface Library.

Features:

* Keyboard teleoperation
* Velocity-based control
* Body-frame velocity commands
* Body-to-NED velocity transformation
* ROS 2 integration with PX4
* Real-time velocity command updates

### trace_square
[PX4 ROS2 Trace Square Autonomously demo](https://youtu.be/qkFvsJ_BtTs)\
\
An autonomous flight mode that generates a square trajectory using velocity setpoints.

Features:

* State-machine based implementation
* Velocity-based navigation
* Autonomous square trajectory generation
* PX4 trajectory setpoint integration

### frame_visualizer
[PX4 ROS2 RViz drone demo](https://youtu.be/_NHdWQzvTgM) \
\
A visualization package used for monitoring vehicle motion and coordinate frames in RViz.

Features:

* Vehicle path visualization
* PX4 NED frame visualization
* PX4 FRD body frame visualization
* Vehicle attitude visualization
* TF broadcasting for RViz

### px4_msgs

ROS 2 message definitions generated from PX4 uORB messages.

### px4_ros2_cpp

PX4 ROS 2 Interface Library used for creating custom flight modes, setpoint types, telemetry interfaces, and autonomous behaviors.

## Coordinate Frames

This project primarily uses PX4-native coordinate frames to maintain consistency with PX4's internal control architecture and avoid unnecessary frame conversions.

### NED Frame (World Frame)

PX4 uses a **North-East-Down (NED)** world frame.

```text id="04rjpn"
X = North
Y = East
Z = Down
```

Example:

```text id="sjx4m2"
+X : Move North
+Y : Move East
+Z : Move Down
```

### FRD Frame (Body Frame)

PX4 uses a **Forward-Right-Down (FRD)** body frame attached to the vehicle.

```text id="7h3p1v"
X = Forward
Y = Right
Z = Down
```

Example:

```text id="y7q7o6"
+X : Move Forward
+Y : Move Right
+Z : Move Down
```

### Body-to-NED Transformation

Velocity commands generated in the body frame can be transformed into the NED frame using the vehicle yaw angle.

```text id="t2s7bw"
Body Velocity
      ↓
Body-to-NED Transform
      ↓
NED Velocity
      ↓
PX4 Trajectory Setpoint
```

This approach is used by the custom teleoperation mode and will also be used in future autonomous control and visual servoing implementations.

### Visualization Frames

The visualization package publishes the following frames:

```text id="lgzow8"
map_ned
    |
    └── base_link_frd
```

Where:

* `map_ned` represents the PX4 world frame
* `base_link_frd` represents the vehicle body frame

These frames can be visualized in RViz using TF displays and coordinate axes.

## PX4 Version Compatibility

This project relies on communication between PX4, ROS 2, `px4_msgs`, and the PX4 ROS 2 Interface Library. To ensure compatibility, all components should use matching PX4 versions.

Using mismatched versions may result in:

* Missing ROS 2 topics
* Message definition mismatches
* Build failures
* Mode registration failures
* Runtime communication issues
* Unexpected flight behavior

### Tested Configuration

This repository has been developed and tested using:

```text id="7zhq0v"
Ubuntu 24.04
ROS 2 Jazzy
PX4-Autopilot v1.17.0
px4_msgs v1.17.0
PX4 ROS 2 Interface Library release/1.17
```

### Cloning Specific Versions

#### PX4-Autopilot

```bash id="fudjlwm"
git clone --recursive --branch v1.17.0 \
https://github.com/PX4/PX4-Autopilot.git
```

#### px4_msgs

```bash id="e1jlwm"
git clone --branch v1.17.0 \
https://github.com/PX4/px4_msgs.git
```

#### PX4 ROS 2 Interface Library

```bash id="z2jlwm"
git clone --branch release/1.17 \
https://github.com/Auterion/px4-ros2-interface-lib.git
```

### Verifying Message Compatibility

Before building the workspace, it is recommended to verify that the installed versions of PX4 and `px4_msgs` are compatible.

Navigate to the PX4 ROS 2 Interface Library:

```bash id="g3jlwm"
cd drone_ws/src/px4-ros2-interface-lib
```

Run the compatibility checker:

```bash id="a4jlwm"
./scripts/check-message-compatibility.py -v \
../px4_msgs \
../../../PX4-Autopilot
```

Expected output:

```text id="b5jlwm"
Compatibility check passed
```

A successful check indicates that the PX4 message definitions and ROS 2 message definitions are compatible.

This check is recommended whenever:

* Upgrading PX4 versions
* Switching PX4 releases
* Updating `px4_msgs`
* Debugging missing ROS 2 topics
* Debugging message definition mismatches

### Migrating to a Different PX4 Version

If upgrading to a newer PX4 release, ensure all related repositories use matching versions.

Example for PX4 v1.18:

```text id="c6jlwm"
PX4-Autopilot             v1.18.x
px4_msgs                  v1.18.x
px4-ros2-interface-lib    release/1.18
```

After updating repositories, perform a clean rebuild:

```bash id="d7jlwm"
cd drone_ws

rm -rf build install log

colcon build
```

## Building the Workspace

Clone the repository:

```bash
git clone --recursive https://github.com/build-error/raven_drone.git
```

Navigate to the ROS 2 workspace:

```bash
cd raven_drone/drone_ws
```

### First-Time Build

The custom flight mode packages depend on `px4_msgs` and `px4_ros2_cpp`.

On a fresh workspace, it is recommended to build these packages first before building the entire workspace.

Skipping this step may lead to:

* Missing dependency errors
* Package discovery failures
* Missing PX4 message definitions
* CMake configuration issues
* Incomplete workspace builds

Build the PX4 dependencies first:

```bash
colcon build \
    --packages-select \
    px4_msgs \
    px4_ros2_cpp
```

Source the workspace:

```bash
source install/setup.bash
```

Build the remaining packages:

```bash
colcon build
```

Source the workspace again:

```bash
source install/setup.bash
```

### Rebuilding After Changes

For normal development:

```bash
colcon build
```

To rebuild a specific package:

```bash
colcon build --packages-select teleop_mode

colcon build --packages-select trace_square

colcon build --packages-select frame_visualizer
```

### Clean Rebuild

If PX4 versions, message definitions, or package dependencies change:

```bash
rm -rf build install log

colcon build
```

Then source the workspace:

```bash
source install/setup.bash
```

> **Note**
>
> Although `colcon build` may work directly on some systems, building `px4_msgs` and `px4_ros2_cpp` first helps avoid dependency-related issues during the initial workspace setup and ensures that the custom PX4 flight mode packages can correctly locate the required message and interface definitions.

## Quick Start

The following steps launch PX4 SITL, establish ROS 2 communication, and start the keyboard teleoperation demo.

### Terminal 1 — PX4 SITL

```bash
cd PX4-Autopilot

make px4_sitl
```

### Terminal 2 — Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 3 — Source ROS 2 Workspace

```bash
cd drone_ws

source install/setup.bash
```

### Terminal 4 — Teleop Flight Mode

```bash
ros2 run teleop_mode teleop_mode
```

### Terminal 5 — Keyboard Controller

```bash
ros2 run teleop_mode teleop_keyboard
```

### Terminal 6 — Frame Visualizer (Optional)

```bash
ros2 run frame_visualizer frame_visualizer
```

### Terminal 7 — RViz (Optional)

```bash
rviz2
```

### QGroundControl

Launch QGroundControl and verify that PX4 SITL is connected.

To fly the drone:

1. Arm the vehicle.
2. Take off.
3. Switch to **Teleop Mode**.
4. Use the keyboard controller to command body-frame velocities.

The drone can now be controlled using the keyboard while all commands are converted from the body frame to the PX4 NED frame before being sent to the flight controller.

## Running the Demo

### Teleoperation Demo

Start the teleoperation flight mode:

```bash id="9t9l6g"
ros2 run teleop_mode teleop_mode
```

The mode should appear in QGroundControl as:

```text id="u0yj93"
Teleop Mode
```

Start the keyboard controller:

```bash id="c9vt3q"
ros2 run teleop_mode teleop_keyboard
```

#### Keyboard Controls

```text id="qq4yjf"
W : Move Forward
S : Move Backward

A : Move Left
D : Move Right

R : Move Up
F : Move Down

Q : Yaw Left
E : Yaw Right

Space : Stop Motion
```

#### Flying the Drone

1. Arm the vehicle using QGroundControl.
2. Take off.
3. Switch to **Teleop Mode**.
4. Use the keyboard controller to command body-frame velocities.

All velocity commands are generated in the vehicle body frame and transformed into the PX4 NED frame before being sent to the flight controller.

---

### Autonomous Square Demo

Start the autonomous square flight mode:

```bash id="n55m4k"
ros2 run trace_square trace_square
```

The mode should appear in QGroundControl as:

```text id="h8cv3f"
Trace Square
```

#### Running the Mission

1. Arm the vehicle.
2. Take off.
3. Switch to **Trace Square** mode.
4. The drone will autonomously fly a square trajectory using velocity setpoints.

This flight mode demonstrates autonomous navigation using a velocity-based state machine.

---

### Frame Visualizer Demo

Start the frame visualizer:

```bash id="c6w03t"
ros2 run frame_visualizer frame_visualizer
```

Launch RViz:

```bash id="7r2lgw"
rviz2
```

Load the saved RViz configuration:

```text id="jjlwm8"
File
 └── Open Config
```

Select the project RViz configuration file.

#### Current Visualizations

* Vehicle path visualization
* PX4 NED world frame visualization
* PX4 FRD body frame visualization
* Vehicle attitude visualization
* TF frame visualization

#### Published Frames

```text id="o0jlwm"
map_ned
    |
    └── base_link_frd
```

Where:

```text id="l3jlwm"
map_ned      → PX4 NED world frame
base_link_frd → PX4 FRD body frame
```

---

### Typical Workflow

```text id="8wjlwm"
Terminal 1 : PX4 SITL

Terminal 2 : MicroXRCEAgent

Terminal 3 : Source ROS Workspace

Terminal 4 : Flight Mode Node

Terminal 5 : Teleop Keyboard (optional)

Terminal 6 : Frame Visualizer (optional)

Terminal 7 : RViz (optional)

QGroundControl : Mission Control
```

## Hardware Platform

The project is intended for both simulation and real-world deployment.

### Simulation Environment

* Ubuntu 24.04
* ROS 2 Jazzy
* PX4 SITL
* QGroundControl
* RViz 2

### Target Hardware

#### Flight Controller

* Pixhawk 6X

#### Airframe

* Holybro X650

#### Companion Computer

* Raspberry Pi 5

#### Communication

* Micro XRCE-DDS
* ROS 2 DDS Middleware

#### Sensors

Planned sensors include:

* Monocular Camera
* RGB-D Camera
* IMU (via PX4)
* GPS (via PX4)

## Useful Resources

* PX4 Documentation: https://docs.px4.io/v1.17/en/
* PX4 ROS 2 User Guide: https://docs.px4.io/v1.17/en/ros2/user_guide/
* PX4 ROS 2 Interface Library: https://docs.px4.io/v1.17/en/ros2/px4_ros2_interface_lib/
* ROS 2 Offboard Control Example: https://docs.px4.io/v1.17/en/ros2/offboard_control/
* PX4 Simulation Documentation: https://docs.px4.io/v1.17/en/simulation/
* Building PX4: https://docs.px4.io/v1.17/en/dev_setup/building_px4.html
* PX4 Autopilot Repository: https://github.com/PX4/PX4-Autopilot
* PX4 ROS 2 Interface Library Repository: https://github.com/Auterion/px4-ros2-interface-lib
* PX4 Message Definitions Repository: https://github.com/PX4/px4_msgs

## License

This project is licensed under the BSD 3-Clause License.

See the [LICENSE](LICENSE) file for details.