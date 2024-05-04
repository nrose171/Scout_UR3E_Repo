#!/usr/bin/env python3

import rospy
from coverage_wp_planner.srv import *
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.transformations import euler_from_quaternion
import time
import math
import numpy as np

def quaternion_parallel_to_xy(quaternion):
    qx, qy, qz = quaternion[0], quaternion[1], quaternion[2]
    z_axis = np.array([0, 0, 1])
    perpendicular_vector = np.cross([qx, qy, qz], z_axis)
    perpendicular_vector /= np.linalg.norm(perpendicular_vector)
    perpendicular_quaternion = np.append(perpendicular_vector, 0)    
    return perpendicular_quaternion



class PoseControllerService:
    def __init__(self):
        rospy.init_node('wp2twist')
        self.odom_pub = rospy.Publisher ('/odom', Odometry, queue_size=1)
        rospy.wait_for_service ('/gazebo/get_model_state')
        self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.current_pose = Pose()
        self.target_pose = Pose()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_service = rospy.Service('wp_2_twist_srv', GoToWP, self.pose_controller_callback)
        self.rate = rospy.Rate(50)  # Control loop frequency

        self.linear_gain = 0.02
        self.angular_gain = 0.2
        # self.angular_diff_gain = 0.05
        self.ki_linear = 0 # 0.02
        self.ki_angular = 0 #0.02
        self.kd_linear = 0 # 0.02
        self.kd_angular = 0 # 0.02
        self.dt = 1

        self.angular_tolerance = 5  # Tolerance for angular error
        self.position_tolerance = 0.1  # Tolerance for position error

    def odom_callback(self):
        odom=Odometry()
        header = Header()
        header.frame_id='/odom'
        model = GetModelStateRequest()
        model.model_name='robot'
        result = self.get_model_srv(model)

        odom.pose.pose = result.pose
        odom.twist.twist = result.twist
        header.stamp = rospy.Time.now()
        odom.header = header

        self.odom_pub.publish(odom)
        self.current_pose = odom.pose.pose
        # print(self.current_pose)
        if self.target_pose == None: self.target_pose = self.current_pose
        orientation_q = odom.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        # print(self.current_pose, self.current_yaw)

    def calculate_errors(self):
        self.odom_callback()
        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y

        ground_curr_pose = quaternion_parallel_to_xy([self.current_pose.orientation.x, self.current_pose.orientation.y,
                                                self.current_pose.orientation.z, self.current_pose.orientation.w])
        ground_target_pose = quaternion_parallel_to_xy([self.target_pose.orientation.x, self.target_pose.orientation.y,
                                            self.target_pose.orientation.z, self.target_pose.orientation.w])

        current_yaw = euler_from_quaternion(ground_curr_pose)[2]
        target_yaw = euler_from_quaternion(ground_target_pose)[2]
        yaw_to_target = math.atan2(dy, dx)
        error_yaw_to_target = 1*(yaw_to_target - current_yaw)
        # print(error_yaw_to_target)
        error_yaw_to_target = math.degrees(math.atan2(math.sin(error_yaw_to_target), math.cos(error_yaw_to_target)))
        print(error_yaw_to_target)
        error_yaw_to_orientation = target_yaw - current_yaw
        error_yaw_to_orientation = math.atan2(math.sin(error_yaw_to_orientation), math.cos(error_yaw_to_orientation))
        error_pos = (dx**2 + dy**2)**0.5
        # print(error_pos, error_yaw_to_target, error_yaw_to_orientation)
        return error_pos, error_yaw_to_target, error_yaw_to_orientation

    def pose_controller_callback(self, req):
        # while not rospy.is_shutdown():
        self.target_pose = req.wp
        rospy.loginfo("Received target pose: {}".format(self.target_pose))

        # Initialize PID variables
        self.integral_pos = 0
        self.integral_yaw_to_target = 0
        self.integral_yaw_to_orientation = 0
        self.previous_error_pos = 0
        self.previous_error_yaw_to_target = 0
        self.previous_error_yaw_to_orientation = 0

        while not rospy.is_shutdown():
            error_pos, error_yaw_to_target, error_yaw_to_orientation = self.calculate_errors()

            ################################ 
            # Calculate PID terms for yaw and position control
            proportional_pos = self.linear_gain * error_pos
            self.integral_pos += error_pos * self.dt
            derivative_pos = (error_pos - self.previous_error_pos) / self.dt
            self.previous_error_pos = error_pos

            # Calculate PID terms for orientation control (yaw_to_target)
            proportional_yaw_to_target = self.angular_gain * error_yaw_to_target
            self.integral_yaw_to_target += error_yaw_to_target * self.dt
            derivative_yaw_to_target = (error_yaw_to_target - self.previous_error_yaw_to_target) / self.dt
            self.previous_error_yaw_to_target = error_yaw_to_target

            # Calculate PID terms for orientation control (yaw_to_orientation)
            proportional_yaw_to_orientation = self.angular_gain * error_yaw_to_orientation
            self.integral_yaw_to_orientation += error_yaw_to_orientation * self.dt
            derivative_yaw_to_orientation = (error_yaw_to_orientation - self.previous_error_yaw_to_orientation) / self.dt
            self.previous_error_yaw_to_orientation = error_yaw_to_orientation

            print(error_pos, error_yaw_to_target)
            if error_pos > self.position_tolerance:
                # If outside the position tolerance, align towards the goal
                if abs(error_yaw_to_target) > self.angular_tolerance:
                    twist = Twist()
                    twist.linear.x = 0
                    twist.angular.z = proportional_yaw_to_target + self.ki_angular * self.integral_yaw_to_target + self.kd_angular * derivative_yaw_to_target
                    self.cmd_vel_pub.publish(twist)
                else:
                    # If aligned towards the goal, move towards it
                    twist = Twist()
                    twist.linear.x = proportional_pos + self.ki_linear * self.integral_pos + self.kd_linear * derivative_pos
                    twist.angular.z = proportional_yaw_to_target + self.ki_angular * self.integral_yaw_to_target + self.kd_angular * derivative_yaw_to_target
                    self.cmd_vel_pub.publish(twist)
            else:
                cmd_vel = Twist()
                self.cmd_vel_publisher.publish(cmd_vel)  # Stop the robot when close to the waypoint
                return reached_wp(True)
            print(cmd_vel)

        # #############################################
        # # Calculate P for yaw and position control
        # last_error_pos, last_error_yaw_to_target, last_error_yaw_to_orientation = self.calculate_errors()
        # while not rospy.is_shutdown():
        #     error_pos, error_yaw_to_target, error_yaw_to_orientation = self.calculate_errors()

        #     if error_pos > self.position_tolerance:
        #         print("Aligned : ", abs(error_yaw_to_target) < self.angular_tolerance, abs(error_yaw_to_target), self.angular_tolerance)
        #         # If outside the position tolerance, align towards the goal
        #         if abs(error_yaw_to_target) > self.angular_tolerance:
        #             twist = Twist()
        #             twist.linear.x = 0
        #             twist.angular.z = self.angular_gain * error_yaw_to_target + (self.angular_diff_gain * (last_error_yaw_to_target - error_yaw_to_target)) 
        #             self.cmd_vel_pub.publish(twist)
        #         else:
        #             # If aligned towards the goal, move towards it
        #             twist = Twist()
        #             twist.linear.x = self.linear_gain * error_pos
        #             twist.angular.z = self.angular_gain/2 * error_yaw_to_target + (self.angular_diff_gain * (last_error_yaw_to_target - error_yaw_to_target))
        #             self.cmd_vel_pub.publish(twist)
        #     else:
        #         # If near the goal, orient according to the target pose
        #         if abs(error_yaw_to_orientation) > self.angular_tolerance:
        #             twist = Twist()
        #             twist.linear.x = 0
        #             twist.angular.z = self.angular_gain * error_yaw_to_orientation
        #             self.cmd_vel_pub.publish(twist)
        #         else:
        #             # Target pose achieved, stop the robot and return True
        #             twist = Twist()
        #             self.cmd_vel_pub.publish(twist)
        #             rospy.loginfo("Target pose achieved!")
        #             return True
        #     last_error_pos, last_error_yaw_to_target, last_error_yaw_to_orientation = error_pos, error_yaw_to_target, error_yaw_to_orientation

        #     self.rate.sleep()


if __name__ == '__main__':
    controller_service = PoseControllerService()
    rospy.spin()