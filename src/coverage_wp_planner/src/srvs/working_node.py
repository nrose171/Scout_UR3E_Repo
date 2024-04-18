#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.transformations import euler_from_quaternion
import numpy as np

class WaypointFollower:
    def __init__(self):
        rospy.init_node('wp2twist')

        self.waypoint_subscriber = rospy.Subscriber('/waypoint', Point, self.waypoint_callback)
        # self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.odom_pub=rospy.Publisher ('/odom', Odometry, queue_size=1)
        rospy.wait_for_service ('/gazebo/get_model_state')
        self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

        self.current_position = Point()
        self.current_yaw = 0.0
        self.target_waypoint = None

        self.linear_vel = 3  # Adjust the linear velocity as needed
        self.angular_vel = 3  # Adjust the angular velocity as needed

        self.rate = rospy.Rate(10)  # 10 Hz

    def waypoint_callback(self, msg):
        self.target_waypoint = msg

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
        self.current_position = odom.pose.pose.position
        if self.target_waypoint == None: self.target_waypoint = self.current_position
        orientation_q = odom.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        print(self.current_position, self.current_yaw)

    def run(self):
        while not rospy.is_shutdown():
            self.odom_callback()
            dx = self.target_waypoint.x - self.current_position.x
            dy = self.target_waypoint.y - self.current_position.y
            dz = self.target_waypoint.z - self.current_position.z

            distance = (dx**2 + dy**2 + dz**2)**0.5

            if distance > 0.5:  # Adjust the distance threshold as needed
                cmd_vel = Twist()
                cmd_vel.linear.x = self.linear_vel * (dx/distance)
                cmd_vel.linear.y = self.linear_vel * (dy/distance)
                cmd_vel.linear.z = self.linear_vel * (dz/distance)
                print(self.current_yaw, np.arctan(dy/dx))
                cmd_vel.angular.z = self.angular_vel * -1*(self.current_yaw - np.arctan(dy/dx))
                self.cmd_vel_publisher.publish(cmd_vel)
            else:
                cmd_vel = Twist()
                self.cmd_vel_publisher.publish(cmd_vel)  # Stop the robot when close to the waypoint
            print(cmd_vel)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = WaypointFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass