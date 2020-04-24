cmake_minimum_required(VERSION 2.8.3)
project(bluetooth_port_manager)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
)

catkin_python_setup()

catkin_package()

install(PROGRAMS src/bluetooth_port_manager/bluetooth_port_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
