#!/usr/bin/env sh
rostopic pub -r 5 /example0/cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[1.0, 0.0, 0.8]' &
rostopic pub -r 5 /example1/cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 1.0, 0.0]' &
