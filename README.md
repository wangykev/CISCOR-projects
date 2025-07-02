# CFW11 ROS 2 Controller

This is a a ROS 2 RQt plugin GUI for controlling and monitoring a WEG CFW-11 motor controller via Modbus RTU (RS-485).

This plugin provides an intuitive graphical interface for setting RPM, starting/stopping the motor, and monitoring live RPM feedback. It interfaces with a ROS 2 backend node that communicates with the drive using pymodbus over a USB-to-RS485 adapter.

---

## Features

- Interactive slider to control motor RPM
- Start/Stop toggle button with visual status feedback
- ROS 2 integration with real-time topic communication
- Compatible with RQt on ROS 2 Jazzy

---

## Requirements

- ROS 2 (tested with Humble or Jazzy)
- Python 3.10+
- [pymodbus](https://github.com/pymodbus-dev/pymodbus)
- USB-RS485 adapter

## Installation

1. Clone the repository into the src directory of your ROS 2 workspace:

```bash
cd ~/treadmill/src  #change treadmill to your actual ros2ws
git clone https://github.com/wangykev/CISCOR-projects.git
```


## How to Run

## Start the backend ROS 2 node that communicates with the CFW-11 over RS-485:
make sure the RS485 USB is plugged in. 

```bash
cd ~/treadmill #change treadmill to your actual ros2ws
source install/setup.bash
ros2 run cfw11_ros2_control treadmill_node.py
```

## Starting the GUI

Make sure to open a new terminal then run:
```bash
cd ~/treadmill #change treadmill to your actual ros2ws
colcon build --packages-select rqt_cfw11_gui
source install/setup.bash
rqt
```
In the RQt menu, go to:

Plugins → cfw11_gui → CFW11 Plugin

You should now see the GUI with:
- A horizontal slider for RPM
- A Start/Stop toggle button
- A live RPM display


## How to Use
The enable button starts/stops the motor and the set speed button sets the speed


## CFW-11 Drive Parameters (for debugging)
| Parameter |      Value       |        Description            |
| --------- | -----------------| ----------------------------- |
| P0220     | 1 (Always REM)   | Always REM mode               |
| P0222     | 9 (serial/usb)   | Speed ref from serial (P0683) |
| P0227     | 2 (serial/usb)   | Run/Stop from serial (P0682)  |


## Notes

If ros2 run fails, make sure you sourced install/setup.bash

You can also run the node directly:
./install/cfw11_ros2_control/bin/treadmill_node
