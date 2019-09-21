# MPPT ROS Package
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![made-with-python](https://img.shields.io/badge/Made%20with-Python-1f425f.svg)](https://www.python.org/)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://github.com/AaronPB/ros_mppt/graphs/commit-activity)
[![Documentation Status](https://readthedocs.org/projects/ansicolortags/badge/?version=latest)](http://wiki.ros.org/ros_mppt)
[![Build Status](http://build.ros.org/job/Kdev__ros_mppt__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__ros_mppt__ubuntu_xenial_amd64/)

ROS package for Victron Energy MPPT devices via VE-Direct Protocol from serial USB

## Introduction
This repository is part of my end-of-degree project as mechanical engineer at University of Almería in Spain.
The aim of this project is to install a PV system into an electric vehicle, creating a data sender from the MPPT device into the ros-system.

## How it works
### MPPT Connection
This software is tested with a [VE.Direct to USB Interface cable](https://www.victronenergy.com.es/accessories/ve-direct-to-usb-interface)

### Topic publisher
At the moment, it sends the most important data from the ve-direct protocol, which are defined in mppt.msg:

 - Battery Voltage (v_bat) - float64
 - Battery Current (i_bat) - float64
 - Solar Panels Voltage (v_pv) - float64
 - Solar Panel Power (p_pv) - float64

## Installing the package
Install the package from the distro repo (kinetic)
```sh

sudo apt-get install ros-kinetic-ros_mppt

```

Or you can also clone the package from github repo into your catkin workspace and build as usual
```sh

$ git clone https://github.com/AaronPB/ros_mppt.git

```

## Customize the sender packets
Depending on the interested values from the serial communication interface, it is possible to define more values into the package.
> Check the [ros_mppt documentation](http://wiki.ros.org/ros_mppt) to learn how to define more values.

**MPPT valid values:**

| Implemented | Label | Units | Description |
| :---: | :---: | :---: | :--- |
| ✓ | V | mV | Main (battery) voltage |
| ✓ | I | mA | Battery current |
| ✓ | VPV | mV | Panel voltage |
| ✓ | PPV | W | Panel power |
| - | PID | - | Product ID |
| - | SER# | - | Serial Number |
| - | HSDS | - | Day sequence number (0...364) |
| - | MPPT | - | Tracker operation mode |
| - | ERR | - | Error code |
| - | CS | - | State of operation |
| - | H19 | 0.01 kW/h | Yield Total (user resettable counter) |
| - | H20 | 0.01 kW/h | Yield today |
| - | H21 | W | Maximum power today |
| - | H22 | 0.01 kW/h | Yield yesterday |
| - | H23 | W | Maximum power yesterday |

## Documentation
### MPPT ROS WIKI

 - [[ROS org] ros_mppt](http://wiki.ros.org/ros_mppt)

### My other repositories for MPPT data management
#### VE Direct MPPT data reader
Visit this repository if you are looking for a ve-direct protocol registration into a file (in this case, in a .xls file)

 - [[Github] VE.Direct MPPT reader](https://github.com/AaronPB/vemppt_reader)

### VE.Direct Protocol Documentation

 - [[PDF] VE.Direct Protocol - Version 3.25](https://www.victronenergy.com.es/download-document/2036/ve.direct-protocol-3.25.pdf)

 - [[PDF] BlueSolar HEX protocol MPPT](https://www.victronenergy.com.es/download-document/4459/bluesolar-hex-protocol-mppt.pdf)
  
 - [[WIKI] MPPT Solar Charger Error Codes](https://www.victronenergy.com/live/mppt-error-codes)
  

## TODO List
 - [x] Custom ros messages sender (mppt topic)
 - [x] Log system
 - [ ] Automatic USB port identification
 - [ ] Disconnection handler with timeout requests

## License
ros_mppt is licensed under the GNU General Public License v3.0, detailed in the [LICENSE](https://github.com/AaronPB/ros_mppt/blob/master/LICENSE) file.
