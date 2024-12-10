#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
import socket
import pickle

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Image

global gnc_position
global gnc_attitude
global client_socket

def initialize():
    # Initialize the socket communication with MRS
    host = '192.168.2.62'  # Replace with Computer 2's IP address
    port = 5000

    # Create socket
    global client_socket
    print("Trying to connect to MRS")
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket made")
    client_socket.connect((host, port))
    print("Socket connected!")

def interface_MRS_with_ROS():

    # Write CV results to these topics for the GNC
    publish_cv_position    = rospy.Publisher('cv/rel_position', Vector3Stamped, queue_size=1)
    publish_cv_orientation = rospy.Publisher('cv/rel_quaternion', QuaternionStamped, queue_size=1)
    publish_cv_bb_centre = rospy.Publisher('cv/bb_centre', String, queue_size=1)

    rospy.init_node('listener', anonymous=True)

    # Receive EKF results & dock-cam images
    rospy.Subscriber('adaptive_gnc/nav/cv/rel_position', Vector3Stamped, update_GNC_position) # EKF position estimate from GNC
    rospy.Subscriber('attitude_nav/cv/rel_quaternion', QuaternionStamped, update_GNC_attitude) # EKF attitude estimate from GNC
    rospy.Subscriber('hw/cam_dock', Image, received_dock_cam_image, callback_args = (publish_cv_position, publish_cv_orientation, publish_cv_bb_centre)) # Dock-cam image

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Start up communication with MRS hardware
    # Send: image, EKF position & attitude estimates.
    # Receive: estimated position, orientation, and bounding box centre

def update_GNC_position(data):
    # We just got a new GNC position update, hold onto it so when we receive a dock-cam image we can send the most up-to-date positioning as well
    global gnc_position
    gnc_position = data.data

def update_GNC_attitude(data):
    # We just got a new GNC attitude update, hold onto it so when we receive a dock-cam image we can send the most up-to-date attitude as well
    global gnc_attitude
    gnc_attitude = data.data

def received_dock_cam_image(data, publish_cv_position, publish_cv_orientation, publish_cv_bb_centre):
    # We just received a dock-cam image! Send it, and the most up-to-date relative state data, to the MRS payload!
    dock_cam_image = data.data
    global gnc_position
    global gnc_attitude

    data = {'dock_cam_image': dock_cam_image, 'ekf_position': gnc_position, 'ekf_attitude': gnc_attitude}
    serialized_data = pickle.dumps(data)  # Serialize the data

    # Send data to the server
    global client_socket
    client_socket.sendall(serialized_data)

    # Wait for response
    response = client_socket.recv(4096)  # Receive up to 4096 bytes
    print(response)
    received = pickle.loads(response)  # Deserialize the response

    print("Response from server:", received)

    # Unpack received data
    cv_rel_position = received['cv_rel_position']
    cv_rel_attitude = received['cv_rel_attitude']
    cv_bb_centre = received['cv_bb_centre']

    # Publish results back to ROS
    publish_cv_position.publish(cv_rel_position)
    publish_cv_orientation.publish(cv_rel_attitude)
    publish_cv_bb_centre.publish(cv_bb_centre)

def test_bridge():
    # This function loads in images from disk and sends them across the bridge for processing
    global client_socket
    for filename in os.listdir(folder_path):
        img_path = os.path.join(folder_path, filename)
        dock_cam_image = Image.open(img_path)

        data = {'dock_cam_image': dock_cam_image, 'ekf_position': gnc_position, 'ekf_attitude': gnc_attitude}
        serialized_data = pickle.dumps(data)  # Serialize the data

        # Send data to MRS
        client_socket.sendall(serialized_data)

        # Wait for response
        response = client_socket.recv(4096)  # Receive up to 4096 bytes
        print(response)
        received = pickle.loads(response)  # Deserialize the response

        print("Response from server:", received)

    print("All images processed!")


if __name__ == '__main__':
    initialize()

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--offline", action="store_true")
    opts = parser.parse_args()

    try:
        if opts.offline:
            test_bridge()
        else:
            interface_MRS_with_ROS()
    except rospy.ROSInterruptException:
        print("Closing socket")
        client_socket.close()  # Close connection
        print("Socket closed, ending")
        pass
