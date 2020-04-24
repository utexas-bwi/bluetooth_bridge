#!/usr/bin/env python

from bluetooth_port_manager import BluetoothPortManager
from serial_bridge import BidirectionalNode
import time
import threading
import rospy
import rospkg

class PortManagerInterface:
    def __init__(self):
        rospy.init_node("port_manager_interface")
        self.registered_clients = dict()
        self.client_threads = dict()

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('serial_bridge')
        
        if not rospy.has_param('/serial_bridge/shared_topics_path'):
            rospy.set_param('shared_topics_path', pkg_path+'/config/demo_topics.yaml')
        self.poll_rate = 1.0 #Hz

    def run(self):
        btpm = BluetoothPortManager()
        btpm.start()
        while not rospy.is_shutdown():
            ports = btpm.get_client_serial_ports()
            if len(ports) > 0:
                if ports[0] not in self.registered_clients.keys():
                    client = BidirectionalNode(port=ports[0])
                    thread = threading.Thread(target=client.run)
                    self.registered_clients[ports[0]] = (client, thread)
                    self.client_threads[client] = thread
                    thread.start()

                # check if port has been closed by bridge
                for port, (client, thread) in self.registered_clients.items():
                    if port.fileno() == -1: # port is closed
                        rospy.loginfo("Found closed port. Closing bridge.")
                        del self.registered_clients[port]
                        btpm.remove_socket(port)
                        client.shutdown_node()
                        thread.join()
                        rospy.loginfo("Bridge closed.")

            rospy.sleep(self.poll_rate)

        for client, thread in self.client_threads.items():
            rospy.loginfo("Shutting down client: ")
            client.shutdown_node()
            thread.join()

        rospy.loginfo("All BidirectionalBridge threads have been stopped.")
        btpm.stop()
        rospy.loginfo("PortManager has been closed. Closing PortManager interface.")

if __name__ == "__main__":
    try:
        pmi = PortManagerInterface()
        pmi.run()
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Exiting...")

