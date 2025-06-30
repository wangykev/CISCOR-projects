# CFW11 ROS 2 Controller

This is a ROS 2 Python package for controlling a WEG CFW-11 VFD over Modbus RTU using a USB-RS485 adapter.

## ✅ Features

- Connects to the CFW-11 using Modbus RTU
- Publishes current motor speed over ROS 2
- Subscribes to run/stop and RPM setpoint commands
- Runs as a standard ROS 2 Python node

## 🛠 Requirements

- ROS 2 (tested with Humble or Jazzy)
- Python 3.10+
- [pymodbus](https://github.com/pymodbus-dev/pymodbus)
- USB-RS485 adapter


## 🚀 How to Run

### 1. Clone and build

```bash
cd ~/CISCOR-projects
source /opt/ros/jazzy/setup.bash  # or humble
colcon build
source install/setup.bash

ros2 run cfw11_ros2_control treadmill_node


Then send commands
ros2 topic pub /cfw11/set_rpm std_msgs/Float32 "data: 20.0"
ros2 topic pub /cfw11/run std_msgs/Bool "data: true"
ros2 topic pub /cfw11/run std_msgs/Bool "data: false"


Monitor actual speed 
ros2 topic echo /cfw11/actual_rpm



🧩 CFW-11 Drive Parameters
| Parameter | Value | Description                   |
| --------- | ----- | ----------------------------- |
| P0220     | 1     | Always REM mode               |
| P0222     | 9     | Speed ref from serial (P0683) |
| P0227     | 2     | Run/Stop from serial (P0682)  |


📦 Notes
If ros2 run fails, make sure you sourced install/setup.bash

You can also run the node directly:
./install/cfw11_ros2_control/bin/treadmill_node
