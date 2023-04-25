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
from std_msgs.msg import Float64MultiArray, Float64, Float32

import numpy as np

# Change this path to your crazyflie-firmware folder
class CrazyflieFollowerLeftNode(Node):

    def __init__(self):
        super().__init__("crazyflie_follower_left")
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, '/cf2/cmd_vel', 10)
        self.odom_subscriber_ = self.create_subscription(
            Odometry, "/cf2/odom", self.odom_callback, 10)
        self.odom_subscriber_lead_ = self.create_subscription(
            Odometry, "/cf1/odom", self.odom_callback_lead, 10)
        self.cmd_vel_subscriber_lead_ = self.create_subscription(
            Twist, '/cf1/cmd_vel', self.cmd_vel_callback, 10)
        self.range_front_subscriber_ = self.create_subscription(
            Range, '/cf2/range_front', self.front_range_callback, 10)
        self.range_back_subscriber_ = self.create_subscription(
            Range, '/cf2/range_back', self.back_range_callback, 10)
        self.range_left_subscriber_ = self.create_subscription(
            Range, '/cf2/range_left', self.left_range_callback, 10)
        self.range_array_publisher_ = self.create_publisher(
            Float64MultiArray, '/cf2/range_val', 10)
        self.offset_subscriber_ = self.create_subscription(
            Float64, '/offset', self.offset_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_to_range_topic)
        self.get_logger().info("Crazyflie follwer left has been created")

        # controller variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.kx = 0.5
        self.ky = 0.5
        self.kt = 0.5
        self.kd = 0.075
        self.kdt = 0.075
        self.xlim = 0.1
        self.ylim = 0.5
        self.tlim = 0.5
        self.theta_err_old = 0.0
        self.x_err_old = 0.0
        self.y_err_old = 0.0
        self.past_time = 0.0

        # leader variables
        self.leader_x = 0.0
        self.leader_y = 0.0
        self.leader_z = 0.0
        self.leader_roll = 0.0
        self.leader_pitch = 0.0
        self.leader_yaw = 0.0

        # follow variables
        self.offset_x = 0.1
        self.offset_y = 0.0
        self.follow_x = 0.0
        self.follow_y = 0.0

        # leader commanded velocities
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_theta = 0.0

        # hold the range values
        self.front = 0.0
        self.back = 0.0
        self.left = 0.0

    def odom_callback(self, odom: Odometry):

        # get time update
        dt =  time.time() - self.past_time

        # get odometry information for the follower
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
        x_err = self.follow_x - self.x
        y_err = self.follow_y - self.y
        theta_err = self.leader_yaw - self.yaw

        # debug
        # self.get_logger().info('xerr: ' + str(x_err))

        # transform error coords to the coord frame of the follower
        rot_array = np.array([np.cos(self.yaw), np.sin(self.yaw), 0, 
                              -np.sin(self.yaw), np.cos(self.yaw), 0, 
                              0, 0, 1])
        rot_matrix = rot_array.reshape((3,3))
        state_array = np.array([float(x_err), float(y_err), 0.0])
        state_matrix = state_array.reshape((3,1))
        err_transform = rot_matrix @ state_matrix

        # simple controller
        x_cmd = self.kx * (err_transform[0]) + self.kd * (err_transform[0] - self.x_err_old)/dt + self.cmd_vel_x
        y_cmd = self.ky * (err_transform[1]) + self.kd * (err_transform[1] - self.y_err_old)/dt + self.cmd_vel_y
        z_rot_cmd = self.kt * (theta_err) + self.kdt * (theta_err - self.theta_err_old)/dt + self.cmd_vel_theta

        # update old var
        self.x_err_old = err_transform[0]
        self.y_err_old = err_transform[1]
        self.theta_err_old = theta_err

        # update time
        self.past_time = time.time()

        # send command to the drone
        cmd = Twist()
        cmd.linear.x=float(x_cmd)
        cmd.linear.y=float(y_cmd)
        cmd.angular.z=float(z_rot_cmd)
        self.cmd_vel_publisher_.publish(cmd)

    def odom_callback_lead(self, odom: Odometry):

        # get the odom for the leader
        leader_q = [0,0,0,0]
        self.leader_x = odom.pose.pose.position.x
        self.leader_y = odom.pose.pose.position.y
        self.leader_z = odom.pose.pose.position.z
        leader_q[0] = odom.pose.pose.orientation.x 
        leader_q[1] = odom.pose.pose.orientation.y
        leader_q[2] = odom.pose.pose.orientation.z
        leader_q[3] = odom.pose.pose.orientation.w
        self.leader_roll, self.leader_pitch, self.leader_yaw = tf_transformations.euler_from_quaternion(leader_q)

        # rotate the desired position with the leader
        # transform desired point to the global frame
        rot_array = np.array([np.cos(self.leader_yaw), np.sin(self.leader_yaw), 0, 
                              -np.sin(self.leader_yaw), np.cos(self.leader_yaw), 0, 
                              0, 0, 1])
        rot_matrix = rot_array.reshape((3,3))
        state_array = np.array([self.offset_x, self.offset_y, 0.0])
        state_matrix = state_array.reshape((3,1))
        err_transform = np.linalg.inv(np.matrix(rot_matrix)) @ state_matrix

        # send the coords of the point to track out
        self.follow_x = err_transform[0] + self.leader_x
        self.follow_y = err_transform[1] + self.leader_y

        # print for debug
        # self.get_logger().info("x: " + str(self.follow_x))
        # self.get_logger().info("y: " + str(self.follow_y))
    
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
    
    def cmd_vel_callback(self, twist: Twist):

        self.cmd_vel_x = twist.linear.x
        self.cmd_vel_y = twist.linear.y
        self.cmd_vel_theta = twist.angular.z

    def front_range_callback(self, range: Range):

        self.front = float(range.range)
    
    def back_range_callback(self, range: Range):

        self.back = float(range.range)

    def left_range_callback(self, range: Range):

        self.left = float(range.range)

    def publish_to_range_topic(self):

        msg = Float64MultiArray()
        msg.data = [self.front, self.back, self.left]
        self.range_array_publisher_.publish(msg)

    def offset_callback(self, msg: Float64):

        self.offset_x = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieFollowerLeftNode()
    rclpy.spin(node)
    rclpy.shutdown()

