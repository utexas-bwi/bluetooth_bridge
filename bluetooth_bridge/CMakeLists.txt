cmake_minimum_required(VERSION 2.8.3)
project(bluetooth_bridge)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(PROGRAMS nodes/serial_node.py nodes/message_info_service.py nodes/bidirectional_serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
