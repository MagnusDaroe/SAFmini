#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from saf.srv import ProcessTimeService
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import pandas as pd
import threading

class Processnode(Node):
    def __init__(self):
        super().__init__('process_node')
        
        # Read data from CSV file
        self.data = pd.read_csv('/home/magnusdaroe/SAFmini/saf/table.csv', sep='\t', index_col=0)

        self.get_logger().info("Read data from CSV file. Ready for requests.")

        # Define service
        self.service = self.create_service(ProcessTimeService, '/processtimeservice', self.process_request)

        # Lock for thread safety
        self.command_lock = threading.Lock()

    def process_request(self, response, request):
        """
        Check if the message is a request and send a reply
        """
        with self.command_lock:
            self.get_logger().info(f"Received request {response}")
            request.process_time = float(self.get_data(response.id_carrier, response.id_station))
            
            # Make reply
            self.get_logger().info(f"Got the following process time from the table {request.process_time}")

            # Publish reply
            return request

    def get_data(self, carrier_id, station_id):
        if carrier_id < 0 or carrier_id > self.data.shape[0] or station_id < 0 or station_id > self.data.shape[1]:
            self.get_logger().info('Invalid carrier_id or station_id')
            return None
        
        return self.data.iloc[carrier_id-1, station_id-1]

def main(args=None):
    rclpy.init(args=args)
    server_node = Processnode()

    rclpy.spin(server_node)

    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
