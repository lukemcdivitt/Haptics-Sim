import rclpy
from rclpy.node import Node
import sys
import math
import serial

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
from std_msgs.msg import Float64MultiArray

import numpy as np

# Change this path to your crazyflie-firmware folder
class CrazyflieControllerNode(Node):

    def __init__(self):
        super().__init__("crazyflie_leader")
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, '/cf1/cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(
            Odometry, "/cf1/odom", self.odom_callback, 10)
        self.range_front_subscriber_ = self.create_subscription(
            Range, '/cf1/range_front', self.front_range_callback, 10)
        self.range_back_subscriber_ = self.create_subscription(
            Range, '/cf1/range_back', self.back_range_callback, 10)
        self.range_vals_subscriber_left_ = self.create_subscription(
            Float64MultiArray, '/cf2/range_val', self.record_left_range, 10)
        self.range_vals_subscriber_right_ = self.create_subscription(
            Float64MultiArray, '/cf3/range_val', self.record_right_range, 10)
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
        x_cmd = 0.5
        y_cmd = 0.5
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
    
    def publish_array(self):
        if len(self.path) > 0:
            msg = MarkerArray()
            msg.markers.clear()
            marker = Marker()
            for elems in self.path:
                marker.pose.position.x = elems[0]
                marker.pose.position.y = elems[1]
                marker.pose.position.z = elems[2]
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                msg.markers.append(marker)
                self.marker_publisher_.publish(msg)

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
    
    def back_range_callback(self, range: Range):

        self.back = range.range

    def record_left_range(self, array: Float64MultiArray):

        self.left = array.data.tolist

    def record_right_range(self, array: Float64MultiArray):

        self.right = array.data.tolist

    def publish_to_remote(self):

        left_vals = self.left
        right_vals = self.right

        # self.get_logger().info('left: ' + str(left_vals))
        # self.get_logger().info('right: ' + str(right_vals))

        data = [self.front, self.back, 
                left_vals[0], left_vals[1], left_vals[2], 
                right_vals[0], right_vals[1], right_vals[2]]
        
        self.get_logger().info(str(data))

def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

