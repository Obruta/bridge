# This code runs on the MRS hardware and receives images, processes them,

import socket
import pickle

def server():
    host = '0.0.0.0'  # Bind to all interfaces
    port = 5000  # Use an available port

    # Create socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)  # Allow 1 connection

    print(f"Server listening on {host}:{port}...")
    conn, addr = server_socket.accept()  # Wait for a client connection
    print(f"Connected by {addr}")

    # Receive data from the client
    data = conn.recv(4096)  # Receive up to 4096 bytes
    received = pickle.loads(data)  # Deserialize the data

    print("Received data:", received)
    # Unpack received data
    image = received['image']
    vector1 = received['vector1']
    vector2 = received['vector2']

    # Perform processing (placeholder)
    vector3 = [x + 1 for x in vector1]  # Example operation
    vector4 = [y * 2 for y in vector2]  # Example operation
    vector5 = [len(image)]  # Example operation

    # Send the response
    response = {'vector3': vector3, 'vector4': vector4, 'vector5': vector5}
    conn.sendall(pickle.dumps(response))

    conn.close()  # Close connection

if __name__ == '__main__':
    server()
