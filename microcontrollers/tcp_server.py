"""
tcp_server.py

A simple TCP server that listens for incoming connections over wifi and prints received data to the console.
This is specifically designed to receive data from a microcontroller over wifi like the RPi Pico 2 W.
"""


import socket
import keyboard
import select
import sys

HOST = '0.0.0.0' # listens for all connections on all interfaces, replace with a specific IP if needed
PORT = 5001 # 5000 is a good default choice, change if needed

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setblocking(False)  # Non-blocking accept
server_socket.bind((HOST, PORT))
server_socket.listen()

print(f"Listening on {HOST}:{PORT}")

conn = None

try:
    while True:
        # use select to check for new connections or data
        readable, _, _ = select.select([server_socket, conn] if conn else [server_socket], [], [], 0.5)

        for s in readable:
            if s is server_socket:
                # accept new connection (non-blocking)
                conn, addr = server_socket.accept()
                conn.setblocking(False)
                print(f"Connected by {addr}")

            elif s is conn:
                data = conn.recv(1024)
                if not data:
                    print("Client disconnected")
                    conn.close()
                    conn = None
                else:
                    print(f"Received: {data.decode()}\n")

except KeyboardInterrupt:
    print("\nExiting server...")

finally:
    if conn:
        conn.close()
    server_socket.close()
    sys.exit(0)
