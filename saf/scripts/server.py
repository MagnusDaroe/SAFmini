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
        self.publish_timer = self.create_timer(0.0, self.server_thread)

        # Flag to indicate reply received
        self.reply_received = False

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('192.168.1.111', 24440))  # You can choose any port that is free on your system
        self.server_socket.listen(10)

        self.connection = False

        self.get_logger().info("Server is waiting for connections...")

        # Lock for thread safety
        self.command_lock = threading.Lock()

    def process_reply(self, msg):
        """
        Callback function for the reply subscriber
        """
        self.get_logger().info(f"Received reply: {msg}")
        with self.command_lock: 
            self.processtime = msg.process_time
            self.reply_received = True

    def server_thread(self):
        """
        Thread function to handle incoming connections
        """
     
        while rclpy.ok():

            if not self.connection:
                client_socket, address = self.server_socket.accept()
                self.get_logger().info(f"Connection from {address} has been established.")
                
                received_data = client_socket.recv(1024)
                self.get_logger().info(f"Received data: {received_data}")
                decoded_data = self.decodeXML(str(received_data))
                self.get_logger().info(f"decoded data: {decoded_data}")


            if decoded_data:
                # Assuming the data has structure "id_carrier, id_station, time_stamp"
                id_station, id_carrier, date, plc_timestamp = decoded_data
                
                msg = Processtime()
                msg.id_carrier = int(id_carrier)
                msg.id_station = int(id_station)
                #msg.plc_timestamp = float(plc_timestamp)
                
                self.reply_received = False
                self.publisher_request.publish(msg)

                while not self.reply_received:
                    pass

                response = f"{self.processtime}"
                client_socket.sendall(response.encode('utf-8')) 

                client_socket.close()
            
                self.connection = False

        self.server_socket.close()


    def decodeXML(self, XML_str):
        PLC_id = XML_str.split("<PLCID>")[1].split("</PLCID>")[0].split("_")[-1]
        carrier_id = XML_str.split("<CARRIERID>")[1].split("</CARRIERID>")[0]
        date_and_time = XML_str.split("<DATEANDTIME>")[1].split("</DATEANDTIME>")[0]
        date_lst = date_and_time.split("#")[1].split("-")
        date= date_lst[0] + "-" + date_lst[1] + "-" + date_lst[2]
        time = date_and_time.split("#")[1].split("-")[-1]
        return [PLC_id, carrier_id, date, time]

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
