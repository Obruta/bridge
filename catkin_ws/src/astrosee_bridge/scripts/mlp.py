#!/usr/bin/env python3

import socket
import pickle
import argparse
import os
import numpy as np
import struct
import cv2
import time

import rospy

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped, Vector3
from geometry_msgs.msg import QuaternionStamped, Quaternion
from sensor_msgs.msg import Image

class Bridge:
    def __init__(self):
        # Initialize the socket communication with MRS
        self.host = '192.168.x.x'
        self.port = 5000

        self.publish_cv_position = None
        self.publish_cv_orientation = None
        self.publish_cv_bb_centre = None

        self.gnc_position = np.array([0.,0.,0.])
        self.gnc_attitude = np.array([0.,0.,0.,1.])
        self.dock_cam_image = None

        self.cv_bridge = CvBridge()

        # Create socket
        print("Making socket for MRS")
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("Socket made")
        while True:
            try:
                print("Trying to connect to MRS...")
                self.client_socket.connect((self.host, self.port))
                print("Socket connected!")
                break
            except socket.timeout as e:
                print("Connection failed, trying again")
                time.sleep(1.0)
                continue

    def test_bridge(self):
        # This function loads in images from disk and sends them across the bridge for processing
        folder_path = 'sample_images/'
        for filename in os.listdir(folder_path):
            img_path = os.path.join(folder_path, filename)
            #print(img_path)
            #dock_cam_image = np.array(PIL_image.open(img_path).convert('L'))
            self.dock_cam_image = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

            #dock_cam_image_chw = np.moveaxis(dock_cam_image, -1, 0)

            print("Checking shape: ", self.dock_cam_image.shape)

            # dummy GNC data
            self.gnc_position = np.array([0.,0.,2.])
            self.gnc_attitude = np.array([0.,0.,0.,1.])
            data = {'camera0': self.dock_cam_image, 'ekf_position': self.gnc_position, 'ekf_attitude': self.gnc_attitude}
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
        #rospy.Subscriber('adaptive_gnc/nav/cv/rel_position', Vector3Stamped, self.update_GNC_position)  # EKF position estimate from GNC
        #rospy.Subscriber('attitude_nav/cv/rel_quaternion', QuaternionStamped,self.update_GNC_attitude)  # EKF attitude estimate from GNC
        rospy.Subscriber('hw/cam_dock', Image, self.received_dock_cam_image, queue_size=1)  # Dock-cam image

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def update_GNC_position(self, data):
        # We just got a new GNC position update, hold onto it so when we receive a dock-cam image we can send the most up-to-date positioning as well

        # Some way to slow down here. rospy.rate or something.

        self.gnc_position = np.array([data.vector.x, data.vector.y, data.vector.z])

    def update_GNC_attitude(self, data):
        # We just got a new GNC attitude update, hold onto it so when we receive a dock-cam image we can send the most up-to-date attitude as well
        self.gnc_attitude = np.array([data.quaternion.x,data.quaternion.y,data.quaternion.z,data.quaternion.w])

    def received_dock_cam_image(self, data):
        # We just received a dock-cam image! Send it, and the most up-to-date relative state data, to the MRS payload!
        self.dock_cam_image = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        # Get the time from the current image
        # time = rospy.Time.now()
        time = data.header.stamp  # taking the timestamp from the dock-cam image when it was created

        print("The FIRST difference between the dock-cam images's time and the rospy time is (queue problems, expected small): " (time - rospy.Time.now()))

        data = {'camera0': self.dock_cam_image, 'ekf_position': self.gnc_position, 'ekf_attitude': self.gnc_attitude}
        serialized_data = pickle.dumps(data)  # Serialize the data

        # Send data to the server
        self.send_in_chunks(serialized_data)

        # Wait for response
        response = self.client_socket.recv(4096)  # Receive up to 4096 bytes
        received = pickle.loads(response)  # Deserialize the response

        print("Response from server:", received)

        # Unpack received data
        cv_rel_position = received['cv_rel_position']
        cv_rel_attitude = received['cv_rel_attitude']
        cv_bb_centre = received['cv_bb_centre']

        position = Vector3Stamped()
        position.header.frame_id = "world"
        position.header.stamp = time
        position.vector = Vector3(x=cv_rel_position[0], y=cv_rel_position[1], z=cv_rel_position[2])

        attitude = QuaternionStamped()
        attitude.header.frame_id = "world"
        attitude.header.stamp = time
        attitude.quaternion = Quaternion(cv_rel_attitude[0], cv_rel_attitude[1], cv_rel_attitude[2], cv_rel_attitude[3])

        bb = Vector3Stamped()
        bb.header.frame_id = "world"
        bb.header.stamp = time
        bb.vector = Vector3(x=cv_bb_centre[0], y=cv_bb_centre[1], z=cv_bb_centre[2])


        # Publish results back to ROS
        self.publish_cv_position.publish(position)
        self.publish_cv_orientation.publish(attitude)
        self.publish_cv_bb_centre.publish(bb)

        print("The SECOND difference between the dock-cam images's time and the rospy time is (processing delays, expected 0.5s): "(time - rospy.Time.now()))

    def send_in_chunks(self, serialized_data, chunk_size=4096):
        """
        Send data in chunks over a socket connection.

        Parameters:
            sock (socket.socket): The socket object.
            data (bytes): The serialized data to send.
            chunk_size (int): The size of each chunk to send.    """

        data_size = len(serialized_data)

        # Send the size of the data as a fixed-size header (4 bytes for size)
        self.client_socket.sendall(struct.pack("!I", data_size))

        print("Sending data size header: ", data_size)

        # Send the serialized data in chunks
        total_sent = 0
        while total_sent < data_size:
            chunk = serialized_data[total_sent:total_sent + chunk_size]
            self.client_socket.sendall(chunk)
            total_sent += len(chunk)

    def close(self):
        print("Closing socket")
        self.client_socket.close()  # Close connection
        print("Socket closed, ending")


if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--offline", action="store_true")
    opts = parser.parse_args()

    bridge = Bridge()

    try:
        if opts.offline:
            bridge.test_bridge()
        else:
            bridge.interface_MRS_with_ROS()
    except rospy.ROSInterruptException:
        bridge.close()
