import socket

HOST = '172.20.66.28'  # Listen on all network interfaces
PORT = 11381      # Choose a port to listen on

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Server listening on {HOST}:{PORT}")

    conn, addr = server_socket.accept()
    print(f"Connection from {addr}")

    with conn:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            inf = b'T#2500ms'
            senddata = inf
            conn.sendall(senddata)
            print(f"Received: {data}")
