#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from saf.msg import Processtime
import socket
import re
import time

class Server(Node):
    def __init__(self):
        super().__init__('listen_node')
        
        # Define subscriber
        self.subscription_reply = self.create_subscription(
            Processtime,
            '/a_reply',
            self.process_reply,
            10
        )

    def process_reply(self, msg):
        """
        Callback function for the reply subscriber
        """
        self.get_logger().info(f"Received reply: {msg}")

def main(args=None):
    rclpy.init(args=args)
    server_node = Server()
    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
