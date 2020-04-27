# The serial_bridge package 

This is a ROS oriented bridge for full-message serial communication between two Linux computers and is based on the rosserial protocol. In fact, this code was based around a [fork](https://github.com/juancamilog/rosserial-mrl/tree/master/rosserial_python) of rosserial_python and has been modified.

The `bluetooth_bridge` is not solely limited to bluetooth connections, and could be used with other PC-to-PC serial connections to transfer ROS messages.

### How does the bridge work?
Two bridges communicate over a virtual serial port to republish remote topics locally, and to publish local topics to the remote bridge. There is no direct concept of `host` and `client`- the bridges reason about Publishers and Subscribers themselves, communicate with the bridge connected to the serial port and setup internal structures to maintain the data.


## Configuring the bridge
So far, the only component of the bridge that you can specify is what local topics you'd like to offer up to the remote bridge.
You should make these changes in the `config/` folder of the package, and structure it similar to the existing `.yaml` configuration files.

Be careful of what topics you specify- if the bluetooth communication gets saturated, the `serial_bridge` communication starts to break down.

## Installation
All files can be correctly installed by invoking the build system for ROS at the root of your catkin workspace:

    $ catkin build

## Using a Unified System
This branch introduces the use of the PortManagerInterface that acts as an intermediary between serial_bridge and bluetooth_port_manager. 

Due to restrictions around the supported Python version in ROS Kinetic and Melodic, the `serial_bridge` MUST be run in a Python2.7 environment. This has not been a problem with Kinetic on 16.04

