import rclpy
from rclpy.node import Node
import sys
import math

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
from std_msgs.msg import Float64MultiArray, Float64, Float32MultiArray, Float32

import numpy as np

# Change this path to your crazyflie-firmware folder
class CrazyflieLeaderNode(Node):

    def __init__(self):
        super().__init__("crazyflie_leader")
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, '/cf1/cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(
            Odometry, "/cf1/odom", self.odom_callback, 10)
        self.range_front_subscriber_ = self.create_subscription(
            Range, '/cf1/range_left', self.front_range_callback, 10)
        self.range_back_subscriber_ = self.create_subscription(
            Range, '/cf1/range_right', self.back_range_callback, 10)
        self.range_vals_subscriber_left_ = self.create_subscription(
            Float64MultiArray, '/cf2/range_val', self.record_left_range, 10)
        self.range_vals_subscriber_right_ = self.create_subscription(
            Float64MultiArray, '/cf3/range_val', self.record_right_range, 10)
        self.range_val_publisher_ = self.create_publisher(
            Float64MultiArray, '/all_ranges', 10)
        self.command_subscriber_ = self.create_subscription(
            Float64MultiArray, '/controller_cmds', self.cmd_callback, 10)
        self.offset_publisher_ = self.create_publisher(
            Float64, '/offset', 10)
        self.timer_ = self.create_timer(0.1, self.publish_to_remote)
        self.get_logger().info("Crazyflie Leader has been created")

        # controller variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.kx = 1.5
        self.ky = 1.5
        self.kt = 1.0
        self.xlim = 0.5
        self.ylim = 0.5
        self.tlim = 2.0
        self.theta_err_old = 0.0
        self.x_err_old = 0.0
        self.y_err_old = 0.0
        self.past_time = 0.0

        # range values
        self.front = 0.0
        self.back = 0.0
        self.left = [0.0, 0.0, 0.0]
        self.right = [0.0, 0.0, 0.0]

        # cmd values
        self.x_cmd = 0.0
        self.y_cmd = 0.0
        self.offset = 0.0

    def odom_callback(self, odom: Odometry):

        # get odometry infor
        q = [0,0,0,0]
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.z = odom.pose.pose.position.z
        q[0] = odom.pose.pose.orientation.x 
        q[1] = odom.pose.pose.orientation.y
        q[2] = odom.pose.pose.orientation.z
        q[3] = odom.pose.pose.orientation.w
        self.roll, self.pitch, self.yaw = tf_transformations.euler_from_quaternion(q)

        # get the info from the controller
        x_cmd = self.x_cmd
        y_cmd = self.y_cmd
        z_rot_cmd = 0.0

        # send command to the drone
        cmd = Twist()
        cmd.linear.x=float(x_cmd)
        cmd.linear.y=float(y_cmd)
        cmd.angular.z=float(z_rot_cmd)
        self.cmd_vel_publisher_.publish(cmd)
    
    def check_distance(self, pt1, pt2):
        ans = np.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)
        return ans

    def limit_velocity(self, vel, limit):
        if vel > limit:
            vel = limit
        elif vel < -limit:
            vel = -limit
        else:
            vel = vel
        return vel
    
    def front_range_callback(self, range: Range):

        self.front = range.range
        print(range.range)
    
    def back_range_callback(self, range: Range):

        self.back = range.range

    def record_left_range(self, array: Float64MultiArray):

        self.left = array.data

    def record_right_range(self, array: Float64MultiArray):

        self.right = array.data

    def publish_to_remote(self):

        left_vals = self.left
        right_vals = self.right

        data = [self.front, self.back, 
                left_vals[0], left_vals[1], left_vals[2], 
                right_vals[2], right_vals[1], right_vals[0]]
        
        for idx,val in enumerate(data):
            if np.abs(val) > 0.6:
                data[idx] = 0.6

        msg = Float64MultiArray()
        msg.data = data
        self.range_val_publisher_.publish(msg)

    def cmd_callback(self, msg: Float64MultiArray):

        if np.abs(msg.data[0]) < 0.1:
            self.y_cmd = 0.0
        else:
            self.y_cmd = msg.data[0]

        if np.abs(msg.data[1]) < 0.1:
            self.x_cmd = 0.0
        else:
            self.x_cmd = msg.data[1]

        self.offset = msg.data[2]

        msg = Float64()
        msg.data = self.offset

        self.offset_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieLeaderNode()
    rclpy.spin(node)
    rclpy.shutdown()

