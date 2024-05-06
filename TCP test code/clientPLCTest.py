import socket
import time

HOST = '192.168.0.101'  # Listen on all network interfaces
PORT = 24440      # Choose a port to listen on


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    print("Connected to server")
    client_socket.connect((HOST, PORT))

    # Data to be sent to the server
    data = b'<CARRIERDATA><PLCID>STPLC_10</PLCID><DATEANDTIME>DT#2024-04-23-10:08:58</DATEANDTIME><CARRIERID>7</CARRIERID></CARRIERDATA>\x00H\xbbl@0\xf8%\x00 \xc2%\x00\x00\x00\x00\x00`\xbbl@(\xc1b@`\xbb\x00'
    
    # Send data to the server
    client_socket.sendall(data)

    # Wait for response from the server
    response = client_socket.recv(1024)
    print("Received response:", response.decode())
        

print("Connection closed")