cmake_minimum_required(VERSION 2.8.3)
project(serial_bridge)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
)

catkin_python_setup()

add_message_files(
    DIRECTORY msg  
    FILES
      Time.msg
)

generate_messages(
      DEPENDENCIES
        std_msgs
)

catkin_package()

install(PROGRAMS nodes/serial_node.py nodes/message_info_service.py nodes/bidirectional_serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
