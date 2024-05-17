#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
#  * Neither the name of SRI International nor the names of its
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
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import String, Float32MultiArray
from moveit_commander.conversions import pose_to_list, list_to_pose

# import scout_ur3e
# from scout_ur3e.srv import *
import ur3e_moveit_config
from ur3e_moveit_config.srv import *
from tf.transformations import *

use_map = True

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


"""
Convenience method for testing if the values in two lists are within a tolerance of each other.
For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
between the identical orientations q and -q is calculated correctly).
@param: goal       A list of floats, a Pose or a PoseStamped
@param: actual     A list of floats, a Pose or a PoseStamped
@param: tolerance  A float
@returns: bool
"""
def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

def rotate_pose(gp, euler):
    # Set goal Pose that is rotated
    rpy = quaternion_from_euler(euler[0], euler[1], euler[2])

    pose_goal = gp.pose

    gp_q = []
    gp_q.append(pose_goal.orientation.x)
    gp_q.append(pose_goal.orientation.y)
    gp_q.append(pose_goal.orientation.z)
    gp_q.append(pose_goal.orientation.w)
    new_q = quaternion_multiply(gp_q, rpy)
    newGP = list_to_pose([pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, new_q[0], new_q[1], new_q[2], new_q[3]])
    print("New GP")
    print(newGP)

    return newGP

class MoveGroupCommander(object):
    
    def __init__(self):
        super(MoveGroupCommander, self).__init__()

        global use_map

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        #  Set planning group 
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Setting Planning Params
        planner = "RRTConnect"
        # plannerPipeline = "ompl"
        ref_frame = "base_link"
        planning_attempts = 100  # planning attempts
        planning_time = 2  # [s] Planning time for computation

        move_group.set_planner_id(planner)
        # move_group.set_planning_pipeline_id(plannerPipeline)

        if use_map is True:
            move_group.set_pose_reference_frame("map")

        move_group.set_end_effector_link("wrist_3_link")
        # move_group.allow_looking(True)
        move_group.allow_replanning(True)
        move_group.clear_path_constraints()
        move_group.clear_trajectory_constraints()
        move_group.set_goal_tolerance(0.1)
        move_group.set_num_planning_attempts(planning_attempts)
        move_group.set_planning_time(planning_time)

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    # Go to state based on joint angles
    def go_to_joint_state(self, jointArr):

        # Set Joint States
        joint_goal = self.move_group.get_current_joint_values()

        print("Heard: ")
        print(jointArr.data)

        for i  in range(6):
            joint_goal[i] = jointArr.data[0]

        # joint_goal[0] = 0
        # joint_goal[1] = -tau / 8
        # joint_goal[2] = 0
        # joint_goal[3] = -tau / 4
        # joint_goal[4] = 0
        # joint_goal[5] = tau / 6  # 1/6 of a turn
        # joint_goal[6] = 0

        # Execute Joint States
        success = self.move_group.go(joint_goal, wait=True)

        # Stop Movement
        self.move_group.stop()

        # Close all controller Tasks
        current_joints = self.move_group.get_current_joint_values()
        all_close(joint_goal, current_joints, 0.01)

        # Return true if within tolerance
        return success

    def go_to_pose_goal(self, gp):
        
        print("First GP")
        print(gp)

        # Create new goal pose
        pose_goal = rotate_pose(gp, [0, pi/2, 0])
        self.move_group.set_pose_target(pose_goal, "wrist_3_link")

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

        # Clear target goal pose
        self.move_group.clear_pose_targets()

        # Close all controller tasks
        current_pose = self.move_group.get_current_pose().pose
        all_close(pose_goal, current_pose, 0.01)
        return success
    
    def attempt_cartesian_path(self, gp):
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        pose_goal = gp.pose
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        waypoints.append(copy.deepcopy(gp))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step  # jump_threshold
        
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

        # Clear target goal pose
        self.move_group.clear_pose_targets()

        # Close all controller tasks
        current_pose = self.move_group.get_current_pose().pose
        
        return all_close(pose_goal, current_pose, 0.01)

        ## END_SUB_TUTORIAL`

def start_JointState_server():
    s = rospy.Service('manip_joint_srv', GoToManipJoints, start_manip_joints)
    print("Waiting for Manip Joint States")
    rospy.spin()

def start_manip_joints(jointArr):
    try:
        print("Starting Manip GP Commander")
        
        commander = MoveGroupCommander()

        if jointArr is not None:
            return commander.go_to_joint_state(jointArr)
        else:
            print("Please pass manipulator goal point")
            
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

def start_GoalPoint_server():
    s = rospy.Service('manip_gp_srv', GoToManipGP, start_manip_gp)
    print("Waiting for Manip Goal Point")
    rospy.spin()


commander = MoveGroupCommander()

def start_manip_gp(gp):
    global commander
    try:
        print("Starting Manip GP Commander")

        if gp is not None:
            return commander.go_to_pose_goal(gp)
            # cartesian = commander.attempt_cartesian_path(gp)
            # print(cartesian)
            # return cartesian
            if cartesian == True:
                return cartesian
            else:
                return commander.go_to_pose_goal(gp)
        else:
            print("Please pass manipulator goal point")
            
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
    
def main():
    global use_map

    rospy.init_node('scout_manip')
    args = sys.argv

    if len(args) > 2 and args[2] == "joint":
        print("Calling Joints")
        start_JointState_server()
    else:
        if args[1] == "true":
            use_map = True
            print("CALLING TRUE")
        elif args[1] == "false":
            use_map = False
            print("CALLING FALSE")
        print("Calling GP")
        start_GoalPoint_server()

if __name__ == "__main__":
    main()