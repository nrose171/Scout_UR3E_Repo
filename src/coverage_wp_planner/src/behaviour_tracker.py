#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.transformations import euler_from_quaternion
import numpy as np

class BehaviourTracker:
    """
        * Load the boat 3d csv
        * Calculate the first waypoint based on the boat 3d csv
          - get to that using wp2twist as a service
        * Select of points that can be reached from current waypoint (in a strip) - perform DFS
        * In that order, visit these poses
        * Capture images, change rviz visualisations, etc.
    """
    def __init__(self):
        rospy.init_node('behaviour_tracker')
        boat_df = pd.read_csv("/root/MM_scout_ur5e_proj/model/model_facets/boat.csv")
        boat_df["x"] += 5
        boat_df["y"] += 5
        boat_df = boat_df.sort_values(['x', 'y', 'z'], ascending=[True, True, True])

        if len(boat_df) > 0:
            next_wp = 0
        else:
            print("There are no points in the given boat.csv")
        self.rate = rospy.Rate(10)  # 10 Hz

    def run(self):
        while not rospy.is_shutdown():
            # call wp2twist service to get to boat_df.iloc[next_wp]
            
            # select points reachable by arm in a strip ==> next_wp is first unvisited point (because sorted)
            # 
            self.rate.sleep()

if __name__ == '__main__':
    try:
        bt = BehaviourTracker()
        bt.run()
    except rospy.ROSInterruptException:
        pass