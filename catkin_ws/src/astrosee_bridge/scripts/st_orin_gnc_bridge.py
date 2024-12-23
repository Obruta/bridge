#!/usr/bin/env python

"""
Must be run in Ubuntu 22+ or something with Python 3.10+
Note: VMs can't use localhost to connect to program on same machine
"""

import socket
import pickle
import argparse
import os
import numpy as np
import struct
import cv2

import rospy

#from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped, Vector3
from geometry_msgs.msg import QuaternionStamped, Quaternion
from sensor_msgs.msg import Image

class Bridge:
    def __init__(self):
        # Initialize the socket communication with ST
        self.st_host = '192.168.2.19'  # Replace with Computer 2's IP address
        self.st_port = 5001

        # Initialize the socket communication with Jetson
        self.jet_host = '192.168.2.62'  # Replace with Computer 2's IP address
        self.jet_port = 5000

        self.publish_cv_position = None
        self.publish_cv_orientation = None
        self.publish_cv_bb_centre = None

        self.gnc_position = np.array([0.,0.,0.])
        self.gnc_attitude = np.array([0.,0.,0.,1.])
        self.dock_cam_image = None

        # Establish sockets
        self.connect_socket_to_st()
        self.connect_socket_to_jet()

        #self.cv_bridge = CvBridge()

    def connect_socket_to_st(self):
        # Create socket to receive images
        self.st_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("ST Socket made")
        while True:
            try:
                print("Trying to connect to ST")
                self.st_socket.connect((self.st_host, self.st_port))
            except Exception as why:
                print("Error connecting to ST: ", why)
                print("Retrying...")
                continue
            print("ST Socket connected!")
            break

    def connect_socket_to_jet(self):
        # Create socket with Jet
        self.jet_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("Jet Socket made")
        while True:
            try:
                print("Trying to connect to Jet")
                self.jet_socket.connect((self.jet_host, self.jet_port))
            except Exception as why:
                print("Error connecting to Jet: ", why)
                print("Retrying...")
                continue
            print("Jet Socket connected!")
            break

    def receive_in_chunks(self):
        # Receive data from Space Teams
        chunk_size = 4096
        print("Trying to receive image from ST...")
        # Receive the 4-byte header containing the data size
        header = self.st_socket.recv(4)
        if len(header) < 4:
            raise RuntimeError("Failed to receive the header")

        # Unpack the header to get the total size of the incoming data
        total_size = struct.unpack("!I", header)[0]
        #print("Total size is: ", total_size)

        # Receive the data in chunks
        data = b""
        while len(data) < total_size:
            chunk = self.st_socket.recv(min(chunk_size, total_size - len(data)))
            if not chunk:
                raise RuntimeError("Connection closed prematurely")
            data += chunk

        # Deserialize the data
        received = pickle.loads(data)
        print("Image received!")
        return received

    def full_bridge_mode(self):

        # KH Dec 20, 2024: This method is currently used to test receiving images from Space Teams
        image_number = 0
        while True:
            try:
                print("Receiving image")
                received_data_from_st = self.receive_in_chunks()
                #print("Received data:", received_data_from_st)
                # Unpack received data
                camera0_image = received_data_from_st['camera0']
                #cv2.imshow("Captured Image from ST", camera0_image)
                cv2.imwrite('camera0_image' + str(image_number) + '.jpg', camera0_image)

                # I've got the image from Space Teams without any black bars!!
                # Now send the image to the Jetson!
                data = {'camera0': camera0_image,'ekf_position': self.gnc_position,'ekf_attitude': self.gnc_attitude}
                serialized_data = pickle.dumps(data)  # Serialize the data

                # Send data to the bridge
                print("Sending image to Jet in chunks!")
                self.send_in_chunks(serialized_data)
                print("Image sent to Jet!")

                # Wait for response
                response = self.jet_socket.recv(4096)  # Receive up to 4096 bytes
                #print(response)
                received = pickle.loads(response)  # Deserialize the response

                print("Response from Jet:", received)

                # Unpack received data
                cv_rel_position = received['cv_rel_position']
                cv_rel_attitude = received['cv_rel_attitude']
                cv_bb_centre = received['cv_bb_centre']


                """time = rospy.Time.now()
    
                position = Vector3Stamped()
                position.header.frame_id = "world"
                position.header.stamp = time
                position.vector = Vector3(x=cv_rel_position[0], y=cv_rel_position[1], z=cv_rel_position[2])
    
                attitude = QuaternionStamped()
                attitude.header.frame_id = "world"
                attitude.header.stamp = time
                attitude.quaternion = Quaternion(cv_rel_attitude[0], cv_rel_attitude[1], cv_rel_attitude[2],
                                                 cv_rel_attitude[3])
    
                bb = Vector3Stamped()
                bb.header.frame_id = "world"
                bb.header.stamp = time
                bb.vector = Vector3(x=cv_bb_centre[0], y=cv_bb_centre[1], z=cv_bb_centre[2])
    
                # Publish results back to ROS
                self.publish_cv_position.publish(position)
                self.publish_cv_orientation.publish(attitude)
                self.publish_cv_bb_centre.publish(bb)"""

                #cv2.waitKey(0)
                image_number = image_number + 1
            except (socket.error, OSError, RuntimeError) as why:
                print("Socket error. Why: ", why)
                try:
                    print("Trying to close ST socket")
                    self.close_st_socket()
                except:
                    pass
                try:
                    print("Trying to close Jet socket")
                    self.close_jet_socket()
                except:
                    pass
                print("Reconnecting sockets")
                self.connect_socket_to_jet()
                self.connect_socket_to_st()
            """except Exception as why:
                print("Non-socket error. Why: ", why)
                print("Continuing")
                continue"""







    def test_bridge_to_jetson(self):

        # This function loads in images from disk and sends them across the bridge for processing
        folder_path = '/data/sample_images/'
        for filename in os.listdir(folder_path):
            img_path = os.path.join(folder_path, filename)
            #dock_cam_image = np.array(PIL_image.open(img_path).convert('L'))
            self.dock_cam_image = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

            #dock_cam_image_chw = np.moveaxis(dock_cam_image, -1, 0)

            print("Checking shape: ", self.dock_cam_image.shape)

            # dummy GNC data
            self.gnc_position = np.array([0.,0.,2.])
            self.gnc_attitude = np.array([0.,0.,0.,1.])
            data = {'dock_cam_image': self.dock_cam_image, 'ekf_position': self.gnc_position, 'ekf_attitude': self.gnc_attitude}
            serialized_data = pickle.dumps(data)  # Serialize the data

            print(len(serialized_data))

            # Send data to MRS
            print("Sending image")
            self.send_in_chunks(serialized_data)
            print("Sent!")
            #client_socket.sendall(serialized_data)

            # Wait for response
            print("Receiving response")
            response = self.client_socket.recv(4096*4)  # Receive up to 4096 bytes
            received = pickle.loads(response)  # Deserialize the response

            print("Response from server:", received)

        print("All images processed!")

    def interface_MRS_with_ROS(self):
        # Start up communication with MRS hardware
        # Send: image, EKF position & attitude estimates.
        # Receive: estimated position, orientation, and bounding box centre

        # Getting topics to write CV results to
        self.publish_cv_position = rospy.Publisher('cv/rel_position', Vector3Stamped, queue_size=1)
        self.publish_cv_orientation = rospy.Publisher('cv/rel_quaternion', QuaternionStamped, queue_size=1)
        self.publish_cv_bb_centre = rospy.Publisher('cv/bb_centre', Vector3Stamped, queue_size=1)

        rospy.init_node('listener', anonymous=True)

        # Subscribe for EKF results & dock-cam images
        rospy.Subscriber('adaptive_gnc/nav/cv/rel_position', Vector3Stamped,
                         self.update_GNC_position)  # EKF position estimate from GNC
        rospy.Subscriber('attitude_nav/cv/rel_quaternion', QuaternionStamped,
                         self.update_GNC_attitude)  # EKF attitude estimate from GNC
        rospy.Subscriber('hw/cam_dock', Image, self.received_dock_cam_image, queue_size=1)  # Dock-cam image

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def update_GNC_position(self, data):
        # We just got a new GNC position update, hold onto it so when we receive a dock-cam image we can send the most up-to-date positioning as well
        self.gnc_position = data.data

    def update_GNC_attitude(self, data):
        # We just got a new GNC attitude update, hold onto it so when we receive a dock-cam image we can send the most up-to-date attitude as well
        self.gnc_attitude = data.data

    def send_in_chunks(self, serialized_data, chunk_size=4096):
        """
        Send data in chunks over a socket connection.

        Parameters:
            sock (socket.socket): The socket object.
            data (bytes): The serialized data to send.
            chunk_size (int): The size of each chunk to send.    """

        data_size = len(serialized_data)

        # Send the size of the data as a fixed-size header (4 bytes for size)
        self.jet_socket.sendall(struct.pack("!I", data_size))

        #print("Sending data size header: ", data_size)

        # Send the serialized data in chunks
        total_sent = 0
        while total_sent < data_size:
            chunk = serialized_data[total_sent:total_sent + chunk_size]
            self.jet_socket.sendall(chunk)
            total_sent += len(chunk)

    def close_st_socket(self):
        print("Closing ST socket")
        self.st_socket.close()  # Close connection
        print("Socket closed!")

    def close_jet_socket(self):
        print("Closing Jet socket")
        self.jet_socket.close()  # Close connection
        print("Socket closed!")


if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--offline", action="store_true")
    opts = parser.parse_args()

    bridge = Bridge()

    try:
        if opts.offline:
            bridge.test_bridge_to_jetson()
        else:
            bridge.full_bridge_mode()
    except rospy.ROSInterruptException:
        bridge.close()
