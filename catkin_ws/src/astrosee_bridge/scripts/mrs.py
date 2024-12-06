# This code runs on the MRS hardware and receives images, processes them,

import socket
import pickle
import numpy as np

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


    while True:
        # Receive data from the client
        data = conn.recv(4096)  # Receive up to 4096 bytes
        received = pickle.loads(data)  # Deserialize the data

        print("Received data:", received)
        # Unpack received data
        dock_cam_image = received['dock_cam_image']
        ekf_position = received['ekf_position']
        ekf_attitude = received['ekf_attitude']

        # Run computer vision on the image, using ekf_position and ekf_attitude to help with modernposit


        ##############################
        ### Do computer vision here###
        ##############################


        # Sample Outputs
        cv_rel_position = np.array([0.,1.,2.])
        cv_rel_attitude = np.array([0.,0.,0.,1.])
        cv_bb_centre = np.array([110,240])

        # Send the response
        print("Sending response")
        response = {'cv_rel_position': cv_rel_position, 'cv_rel_attitude': cv_rel_attitude, 'cv_bb_centre': cv_bb_centre}
        conn.sendall(pickle.dumps(response))
        print("response sent!")

    #conn.close()  # Close connection

if __name__ == '__main__':
    server()

