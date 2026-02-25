# Cavalier System: Forklift Controls

This repository contains the core ROS 2 control stack for the autonomous forklift system. It is responsible for translating high-level navigation and teleoperation commands into safe, physical hardware actuation via the CAN bus.



## System Architecture

The controls stack is designed around a **Gatekeeper Architecture** to ensure absolute safety before any hardware is energized. The data flow strictly follows this pipeline:

1. **Input Generation:** Teleop nodes (physical joystick or Adamo web interface) generate raw movement intent.
2. **Safety & Multiplexing:** The Safety Node intercepts all movement intents. It verifies network heartbeats, watchdog timers, and deadman switches. 
3. **Hardware Translation:** If the command is deemed safe, it is passed to the Driver Node. The driver applies software limits (e.g., mast height restrictions) and dynamic scaling presets before translating the normalized commands into CANOpen RPDOs.
4. **Hardware Execution:** The CAN interface pushes the commands to the Curtis Drive Controller and hydraulic valve block.

---

## Packages Overview

Detailed documentation for each package can be found in their respective `README.md` files.

### 1. `forklift_msgs`
Contains the custom ROS 2 message interfaces used across the stack, most notably the `ForkliftDirectCommand` message which standardizes drive, steer, lift, tilt, and side-shift requests.

### 2. `forklift_teleop`
Handles raw input from human operators. 
* Translates physical gamepad inputs (`sensor_msgs/Joy`) into standardized forklift commands.
* Manages gear shifting (Forward/Reverse) and applies directional inversions for intuitive rear-wheel steering.

### 3. `forklift_safety`
The central safety multiplexer and watchdog.
* Monitors the `/safety` heartbeat from the web teleop (catching unfocused windows or high latency).
* Enforces strict timeout limits on incoming commands.
* Instantly clamps all outputs to `0.0` if any upstream node fails or the network drops.

### 4. `forklift_driver`
The hardware interface layer for the MBV15 forklift.
* **Driver Node:** Applies operational presets (e.g., `default`, `turtle`, `transit`), scales joystick inputs, and enforces software bounds using feedback from the SICK draw wire encoder.
* **CAN Interface (`mbv15_iface`):** A pure Python hardware abstraction layer. Manages the CANOpen socket, sets critical hydraulic interlock flags, and strictly sequences the Curtis controller's RPDO and TPDO messages for safe motor and pump actuation.

---

## Getting Started

### Building the Workspace
From the root of your controls workspace:
```bash
colcon build --symlink-install
source install/setup.bash