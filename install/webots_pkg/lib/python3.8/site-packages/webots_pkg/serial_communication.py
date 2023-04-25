import rclpy
from rclpy.node import Node
import sys
import math
import struct
import time

from std_msgs.msg import String
import time
from geometry_msgs.msg import Pose
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float64MultiArray, Float32MultiArray
import serial
from scipy.interpolate import interp1d

import numpy as np

# Change this path to your crazyflie-firmware folder
class SerialCommunicationNode(Node):

    def __init__(self):
        super().__init__("serial_communication")
        self.range_val_subscriber_ = self.create_subscription(
            Float64MultiArray, '/all_ranges', self.range_callback, 10)
        self.cmds_publisher_ = self.create_publisher(
            Float64MultiArray, '/controller_cmds', 10)
        self.timer_ = self.create_timer(0.01, self.send_to_device)
        self.device = serial.Serial('/dev/ttyACM0', baudrate=9600)

        self.data_from_controller_array = [0.0, 0.0, 0.0]
        # 0 - x cmd
        # 1 - y cmd
        # 2 - spacing cmd

        self.data_to_controller_array = [1, 1, 1, 1, 1, 1, 1, 1] # now in cm
        # 0  - leader front
        # 1 - leader back
        # 2 - left front
        # 3 - left back
        # 4 - left left
        # 5 - right front 
        # 6 - right back 
        # 7 - right right

        self.func = interp1d([0,350],[1,8])

        self.get_logger().info("Serial Communication Running")


    def range_callback(self, array: Float64MultiArray):

        new_array = []
        for elem in array.data:
            new_array.append(int(self.func(elem * 100)))
        self.data_to_controller_array = new_array

    def send_to_device(self):

        # for idx,elems in enumerate(self.data_to_controller_array):
        #     self.device.write(str(elems).encode('utf-8'))
        #     print(str(elems).encode('utf-8'))

        print(self.data_to_controller_array)
        data = bytearray(self.data_to_controller_array)
        self.device.write(data)

        read_fail = True

        curr_time = time.time()
        while self.device.in_waiting < 12:
            if time.time()- curr_time > 2:
                cmds_unpack_roll = 0.0
                cmds_unpack_pitch = 0.0
                cmds_offset = 0.0
                read_fail = False
                break
            pass

        if read_fail:
            cmds_roll = self.device.read(4)
            cmds_unpack_roll = struct.unpack('f', cmds_roll)
            cmds_pitch = self.device.read(4)
            cmds_unpack_pitch = struct.unpack('f', cmds_pitch)
            cmds_offset = self.device.read(4)
            cmds_unpack_offset = struct.unpack('f', cmds_offset)

            if cmds_unpack_roll[0] < -3:
                cmds_unpack_roll = (cmds_unpack_roll[0] - -6) / -1
            else:
                cmds_unpack_roll = 1 - (cmds_unpack_roll[0] - -3) / 1

            # self.get_logger().info(str(cmds_unpack_roll))
            # self.get_logger().info(str(cmds_unpack_pitch[0]))
            # self.get_logger().info(str(cmds_unpack_offset[0]))

            data_new = [cmds_unpack_roll, cmds_unpack_pitch[0], cmds_unpack_offset[0]]
        else:
            data_new = [0.0, 0.0, 0.0]

        msg = Float64MultiArray()
        msg.data = data_new

        self.cmds_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommunicationNode()
    rclpy.spin(node)
    rclpy.shutdown()

    
