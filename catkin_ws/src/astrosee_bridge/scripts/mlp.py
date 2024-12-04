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
from sensor_msgs.msg import CompressedImage

global gnc_position
global gnc_attitude
global client_socket

def initialize():
    # Initialize the socket communication with MRS
    host = '192.168.1.2'  # Replace with Computer 2's IP address
    port = 5000

    # Create socket
    global client_socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))

def interface_MRS_with_ROS():

    # Write CV results to these topics for the GNC
    publish_cv_position    = rospy.Publisher('cv/rel_position', Vector3Stamped, queue_size=1)
    publish_cv_orientation = rospy.Publisher('cv/rel_quaternion', QuaternionStamped, queue_size=1)
    publish_cv_bb_centre = rospy.Publisher('cv/bb_centre', String, queue_size=1)

    rospy.init_node('listener', anonymous=True)

    # Receive EKF results & dock-cam images
    rospy.Subscriber('adaptive_gnc/nav/cv/rel_position', Vector3Stamped, update_GNC_position) # EKF position estimate from GNC
    rospy.Subscriber('attitude_nav/cv/rel_quaternion', QuaternionStamped, update_GNC_attitude) # EKF attitude estimate from GNC
    rospy.Subscriber('mgt/img_sampler/dock_cam/image_record', CompressedImage, received_dock_cam_image) # Dock-cam image

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Start up communication with MRS hardware
    # Send: image, EKF position & attitude estimates.
    # Receive: estimated position, orientation, and bounding box centre


if __name__ == '__main__':
    initialize()
    received_dock_cam_image()
    try:
        interface_MRS_with_ROS()
    except rospy.ROSInterruptException:
        client_socket.close()  # Close connection
        pass

def update_GNC_position(data):
    # We just got a new GNC position update, hold onto it so when we receive a dock-cam image we can send the most up-to-date positioning as well
    global gnc_position
    gnc_position = data.data

def update_GNC_attitude(data):
    # We just got a new GNC attitude update, hold onto it so when we receive a dock-cam image we can send the most up-to-date attitude as well
    global gnc_attitude
    gnc_attitude = data.data

def received_dock_cam_image(data):
    # We just received a dock-cam image! Send it, and the most up-to-date relative state data, to the MRS payload!
    dock_cam_image = data.data


    # Blocking wait for the results
    # Prepare data to send
    image = [255] * (28 * 28)  # Example image as a flat array
    vector1 = [1, 2, 3]
    vector2 = [4, 5, 6]

    data = {'image': image, 'vector1': vector1, 'vector2': vector2}
    serialized_data = pickle.dumps(data)  # Serialize the data

    # Send data to the server
    client_socket.sendall(serialized_data)

    # Wait for response
    response = client_socket.recv(4096)  # Receive up to 4096 bytes
    received = pickle.loads(response)  # Deserialize the response

    print("Response from server:", received)

    # Publish results back to ROS


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()