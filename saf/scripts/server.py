#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from saf.msg import Processtime
import socket
import re

class Server(Node):
    def __init__(self):
        super().__init__('server_node')
        
        # Define subscriber
        self.subscription_reply = self.create_subscription(
            Processtime,
            '/reply',
            self.process_reply,
            10
        )

        # Define publisher
        self.publisher_request = self.create_publisher(
            Processtime,
            '/request',
            10
        )
        self.publish_timer = self.create_timer(0.0, self.status_publisher)

        # Flag to indicate reply received
        self.reply_received = False

        # Lock for thread safety
        self.command_lock = threading.Lock()

    def process_reply(self, msg):
        """
        Callback function for the reply subscriber
        """
        with self.command_lock: 
            self.get_logger().info(f"Received reply: {msg.processtime}")
            self.processtime = msg.processtime
            self.reply_received = True

    def server_thread(self):
        """
        Thread function to handle incoming connections
        """
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('172.20.66.36', 43108))  # You can choose any port that is free on your system
        server_socket.listen(10)

        self.get_logger().info("Server is waiting for connections...")

        while rclpy.ok():
            client_socket, address = server_socket.accept()
            self.get_logger().info(f"Connection from {address} has been established.")
            
            received_data = client_socket.recv(1024)
            decoded_data = self.decode(received_data)

            if decoded_data:
                # Assuming the data has structure "id_carrier, id_station, time_stamp"
                id_carrier, id_station, plc_timestamp = decoded_data
                
                msg = Processtime()
                msg.id_carrier = id_carrier
                msg.id_station = id_station
                msg.plc_timestamp = plc_timestamp
                
                self.reply_received = False
                self.publisher_request.publish(msg)

                while not self.reply_received:
                    pass

                response = f"{self.processtime}"
                client_socket.sendall(response.encode('utf-8')) 

            client_socket.close()

        server_socket.close()

    def decode(self, data):
        """
        Decode received data
        """
        match = re.search(rb'^(\d+), (\d+), (\d+)', data)
        if match:
            return tuple(int(match.group(i)) for i in range(1, 4))
        else:
            self.get_logger().info("No match found.")
            return None

def main(args=None):
    rclpy.init(args=args)
    server_node = Server()
    server_thread = threading.Thread(target=server_node.server_thread, daemon=True)
    server_thread.start()
    rclpy.spin(server_node)
    server_thread.join()
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
