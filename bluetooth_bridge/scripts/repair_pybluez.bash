#!/usr/bin/env bash
source ~/catkin_ws/devel/setup.bash
roscd bluetooth_bridge/scripts/
sudo cp repaired_init.py /usr/local/lib/python2.7/dist-packages/bluetooth/__init__.py
