# MPPT ROS Package
ROS package for Victron Energy MPPT devices via VE-Direct Protocol from serial USB

## Introduction
This repository is part of my end-of-degree project as mechanical engineer at University of Almería in Spain.
The aim of this project is to install a PV system into an electric vehicle, creating a data sender from the MPPT device into the ros-system.

## MPPT Connection
This software is tested with a [VE.Direct to USB Interface cable](https://www.victronenergy.com.es/accessories/ve-direct-to-usb-interface)

## How it works
At the moment, it sends the most important data from the ve-direct protocol, which are defined in mppt.msg:

 - Battery Voltage (v_bat) - float64
 - Battery Current (i_bat) - float64
 - Solar Panels Voltage (v_pv) - float64
 - Solar Panel Power (p_pv) - float64

Run roscore and vemppt_ros.py

## Documentation
### MPPT ROS WIKI
In progress :)

 - [[WIKI] ros_mppt](http://wiki.ros.org/ros_mppt)

### My other repositories for MPPT data management
#### VE Direct MPPT data reader
Visit this repository if you are looking for a ve-direct protocol registration into a file (in this case, a .xls file)

 - [[Github] VE.Direct MPPT reader](https://github.com/AaronPB/vemppt_reader)

### VE.Direct Protocol Documentation

  - [[PDF] VE.Direct Protocol - Version 3.25](https://www.victronenergy.com.es/download-document/2036/ve.direct-protocol-3.25.pdf)

  - [[PDF] BlueSolar HEX protocol MPPT](https://www.victronenergy.com.es/download-document/4459/bluesolar-hex-protocol-mppt.pdf)
  
  - [[WIKI] MPPT Solar Charger Error Codes](https://www.victronenergy.com/live/mppt-error-codes)
