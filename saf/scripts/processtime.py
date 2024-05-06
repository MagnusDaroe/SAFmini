#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from saf.msg import Processtime
import pandas as pd

class Procesnode(Node):
    def __init__(self):
        super().__init__('process_node')
        
        # Define subscriber
        self.subscription_reply = self.create_subscription(
            Processtime,
            '/request',
            self.process_request,
            10
        )

        # Define publisher
        self.publisher_request = self.create_publisher(
            Processtime,
            '/a_reply',
            10
        )

        # Read data from CSV file
        self.data = pd.read_csv('/home/magnusdaroe/SAFmini/saf/table.csv', sep='\t', index_col=0) # Make sure to change the path to the correct one

        self.get_logger().info("Read data from CSV file. Ready for requests.")

        # Lock for thread safety
        self.command_lock = threading.Lock()

    def process_request(self, msg):
        """
        Check if the message is a request and send a reply
        """


        with self.command_lock: 
            self.get_logger().info(f"Received request {msg}")
            self.id_carrier = msg.id_carrier
            self.id_station = msg.id_station 
            
            # Make reply
            self.processtime =self.get_data(self.id_carrier, self.id_station)
            self.get_logger().info(f"Got the following process time from the table {self.processtime}")

            # Publish reply
            reply_msg = Processtime()
            reply_msg.process_time = float(self.processtime)

            self.publisher_request.publish(reply_msg)

    def get_data(self, carrier_id, station_id):
        if carrier_id < 0 or carrier_id >= self.data.shape[0] or station_id < 0 or station_id >= self.data.shape[1]:
            print('Invalid carrier_id or station_id')
            return None
        
        return self.data.iloc[carrier_id-1, station_id-1]

def main(args=None):
    rclpy.init(args=args)
    server_node = Procesnode()
    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
