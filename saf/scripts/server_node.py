#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from saf.srv import ProcessTimeService
import socket
import threading
import time
import csv

class Server(Node):
    def __init__(self):
        super().__init__('server_node')

        self.processtime = None  # Initialize processtime attribute

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('172.20.66.80', 24440))  # You can choose any port that is free on your system
        self.server_socket.listen(10)
        
        # Open the .csv file to write the data. Make a new file if it does not exist.
        self.csv_file = open('/home/magnusdaroe/SAFmini/saf/process_times.csv', 'a')
        self.csv_writer = csv.writer(self.csv_file)

        self.get_logger().info("Server is waiting for connections...")
        self.client_socket, self.address = self.server_socket.accept()
        self.get_logger().info(f"Connection from {self.address} has been established.")

        

    def server_thread(self):
        """
        Thread function to handle incoming connections
        """
        while rclpy.ok():
            try:
                self.data_decoded = False
                received_data = None
                decoded_data = None
                
                received_data = self.client_socket.recv(1024)
  
                if not received_data == b'':   
                    decoded_data = self.decodeXML(str(received_data))
                    self.get_logger().info(f"Received data, decoded as: {decoded_data}")
                else: 
                    self.get_logger().info(f"Received bad data:{received_data}")
                    time.sleep(1)
                    # Close connection
                    self.client_socket.close()
                    # Try to get connection again
                    self.client_socket, self.address = self.server_socket.accept()
                    self.get_logger().info(f"Connection from {self.address} has been established.")
                    
                if decoded_data:
                    id_station, id_carrier, date, plc_timestamp = decoded_data
                    
                    self.client = self.create_client(ProcessTimeService, '/processtimeservice')

                    while not self.client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('service not available, waiting again...')

                    request = ProcessTimeService.Request()
                    request.id_station = int(id_station)
                    request.id_carrier = int(id_carrier)

                    future = self.client.call_async(request)

                    # Custom loop to wait for future to be done
                    while rclpy.ok() and not future.done():
                        time.sleep(0.1)  # Adjust sleep time as needed
                    if future.done():
                        if future.result() is not None:
                            self.processtime = future.result().process_time
                            self.get_logger().info(f"Got the following process time from the table {self.processtime}")
                            response = f"T#{int(self.processtime)}ms"
                        else:
                            self.get_logger().error('Service call failed %r' % (future.exception(),))
                            response = "Service call failed"
                    else:
                        self.get_logger().error('Service call timed out')

                    self.get_logger().info(f"Received reply. Sending {response.encode('utf-8')}")
                    self.client_socket.sendall(response.encode('utf-8')) 

                    # Log information in a .csv file
                    self.csv_writer.writerow([id_station, id_carrier, date, plc_timestamp, self.processtime])
                    self.csv_file.flush()
                    self.get_logger().info(f"Data has been written to the .csv file")

                    self.get_logger().info(f"Ready for next connection!\n\n")

            except BrokenPipeError:
                self.get_logger().info("Connection was closed by the client. Waiting for new connections...")
                time.sleep(2)
                self.client_socket, self.address = self.server_socket.accept()
                self.get_logger().info(f"Connection from {self.address} has been established.")


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
    server_node.get_logger().info("Server node is running")


    server_thread = threading.Thread(target=server_node.server_thread)
    server_thread.start()

    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
