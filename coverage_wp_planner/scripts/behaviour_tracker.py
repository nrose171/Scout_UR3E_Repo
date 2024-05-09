#!/usr/bin/env python3

import logging
logging.basicConfig(format='[%(filename)s:%(lineno)d] %(message)s', level=logging.INFO)
import time
import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from scipy.spatial import ConvexHull
from coverage_wp_planner.srv import *

# Move Base Client
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Manipulator Client
from ur3e_moveit_config.srv import GoToManipGP, GoToManipJoints

import networkx as nx

rospack = rospkg.RosPack()
data_path = rospack.get_path('object_spawner') +"/gazebo_resources/"
 
def create_graph_perform_dfs(pose_df, comp_boat_df, arm_base_point, max_dist_node_idx):
    print(pose_df)
    pose_df_idx = pose_df.index
    G = nx.Graph()
    # Add nodes to the graph
    for index, row in pose_df.iterrows():
        # print(index, row)
        G.add_node(index, pos=(row['x'], row['y']))

    

    # Add edges between adjacent points
    for i in range(len(pose_df)):
        for j in range(i+1, len(pose_df)):
            if (abs(pose_df.iloc[i]['x'] - pose_df.iloc[j]['x']) <= 1 and \
                abs(pose_df.iloc[i]['y'] - pose_df.iloc[j]['y']) <= 1 and \
                abs(pose_df.iloc[i]['z'] - pose_df.iloc[j]['z']) <= 1):
                print(pose_df_idx[i], pose_df_idx[j])
                G.add_edge(pose_df_idx[i], pose_df_idx[j])

    dfs_node_order = list(nx.dfs_preorder_nodes(G, source=max_dist_node_idx))
    print(dfs_node_order)
    # print(pose_df['x'][dfs_node_order])
    # print(pose_df['y'][dfs_node_order])

    fig = plt.figure()
    ax_3d = fig.add_subplot(111, projection='3d')
    ax_3d.scatter(comp_boat_df['x'].values, 
                  comp_boat_df['y'].values, 
                  comp_boat_df['z'].values, marker='o', color="r")
    ax_3d.plot(pose_df['x'][dfs_node_order].values, 
                  pose_df['y'][dfs_node_order].values, 
                  pose_df['z'][dfs_node_order].values, marker='^', color="b")
    ax_3d.scatter(arm_base_point[0], arm_base_point[1], arm_base_point[2], marker='o', color="g")

    ax_3d.set_xlim3d(1, 6)
    ax_3d.set_ylim3d(-1, 1)
    ax_3d.set_zlim3d(0, 1)

    plt.pause(5)
    plt.show(block=False)

    # Perform DFS on the graph
    print("Depth-First Search (DFS) traversal:")
    return dfs_node_order
    



def process_pose_array(poses, dist, horizontal=False, arm_base_height=0.25,num_skip=1):   # Distance to shift the points (in meters)
    processed_pose_arr = PoseArray()
    processed_pose_arr.header.frame_id = 'map'
    processed_pose_arr.header.stamp = rospy.Time.now()

    flip_matrix = np.array([
        [-1, 0, 0],
        [0, 1, 0],
        [0, 0, -1]
    ])

    for i in range(0, len(poses), num_skip):
        # Convert quaternion to rotation matrix
        q = poses.iloc[i][["qx", "qy", "qz", "qw"]].values
        rot_matrix = np.array([
            [1 - 2*q[1]**2 - 2*q[2]**2, 2*q[0]*q[1] - 2*q[2]*q[3], 2*q[0]*q[2] + 2*q[1]*q[3]],
            [2*q[0]*q[1] + 2*q[2]*q[3], 1 - 2*q[0]**2 - 2*q[2]**2, 2*q[1]*q[2] - 2*q[0]*q[3]],
            [2*q[0]*q[2] - 2*q[1]*q[3], 2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[0]**2 - 2*q[1]**2]
        ])

        # Calculate the direction vector from the rotation matrix
        direction = rot_matrix[:, 0]  # First column of the rotation matrix

        # Shift the point by the specified distance in the direction of the quaternion
        shifted_point = poses.iloc[i][["x", "y", "z"]].values + dist * direction
        # direction = t3d.mat2quat(rot_matrix[:, 0].T)

                # Multiply the rotation matrix by the flip matrix
        flipped_matrix = np.dot(rot_matrix, flip_matrix)

        # Convert the flipped rotation matrix back to a quaternion
        flipped_quat = quaternion_from_matrix(flipped_matrix)
        flipped_quat /= np.linalg.norm(flipped_quat)

        if horizontal:
            shifted_point[2] = arm_base_height
            flipped_quat = quat_horiz_parallel_to_surface(flipped_quat)

        pose = Pose()
        pose.position.x = shifted_point[0]
        pose.position.y = shifted_point[1]
        pose.position.z = shifted_point[2]
        pose.orientation.x = flipped_quat[0]
        pose.orientation.y = flipped_quat[1]
        pose.orientation.z = flipped_quat[2]
        pose.orientation.w = flipped_quat[3]
        processed_pose_arr.poses.append(pose)

    return processed_pose_arr

def quaternion_from_matrix(matrix):
    # Extract the rotation matrix components
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = matrix.flatten()

    # Calculate the quaternion components
    q0 = np.sqrt(max(0, 1 + m00 + m11 + m22)) / 2
    q1 = np.sqrt(max(0, 1 + m00 - m11 - m22)) / 2
    q2 = np.sqrt(max(0, 1 - m00 + m11 - m22)) / 2
    q3 = np.sqrt(max(0, 1 - m00 - m11 + m22)) / 2

    # Determine the signs of the quaternion components
    q1 = q1 * np.sign(m21 - m12)
    q2 = q2 * np.sign(m02 - m20)
    q3 = q3 * np.sign(m10 - m01)

    return np.array([q1, q2, q3, q0])

def quat_horiz_parallel_to_surface(quaternion):
    qx, qy, qz = quaternion[0], quaternion[1], quaternion[2]
    z_axis = np.array([0, 0, 1])
    perpendicular_vector = np.cross([qx, qy, qz], z_axis)
    perpendicular_vector /= np.linalg.norm(perpendicular_vector)
    perpendicular_quaternion = np.append(-1 * perpendicular_vector, 0)
    return perpendicular_quaternion

def get_mesh(filename):
    mesh = Marker()
    mesh.type = Marker.MESH_RESOURCE
    mesh.mesh_resource = filename
    mesh.scale.x = 1
    mesh.scale.y = 1
    mesh.scale.z = 1
    mesh.color.r = 0
    mesh.color.g = 0
    mesh.color.b = 1
    mesh.color.a = 1
    mesh.header.frame_id = 'map'
    mesh.header.stamp = rospy.Time.now()
    return mesh

def check_point_in_hull(inp_points, hull, eps=0, print_eps=False):
    # A is shape (f, d) and b is shape (f, 1).
    A, b = hull.equations[:, :-1], hull.equations[:, -1:]
    def contained(x): 
        if print_eps: 
            thresh_eps = np.asarray(x) @ A.T + b.T
            print([np.min(thresh_eps), np.max(thresh_eps)])
            print(np.all(np.asarray(x) @ A.T + b.T < eps, axis=1))
        return np.all(np.asarray(x) @ A.T + b.T < eps, axis=1)
    return contained(inp_points)


# Call the move base client for a goal pose 
# REQUIRES only z and w of orientation to be set
def call_movebase_service(goal_pose):

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
 
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = goal_pose

    # Sends the goal to the action server.
    client.send_goal(goal)
    wait = client.wait_for_result()

    # Failure to reach server
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_goal_status_text()

# Expected input: Pose w.r.t. map coordinate frame
def call_manipulator_service(goal_pose):

    # Wait for the manipulator service
    rospy.wait_for_service('manip_gp_srv')
    try:

        # Push goal pose message
        push_msg = rospy.ServiceProxy('manip_gp_srv', GoToManipGP)
        response = push_msg(goal_pose)
        return response

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



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
        # data_path = "/root/Scout_UR3E_Repo/src/object_spawner/gazebo_resources"
        self.boat_df = pd.read_csv(f"{data_path}/model_facets/boat.csv")
        self.mesh = get_mesh(f'{data_path}/models/boat/meshes/boat.dae')
        self.mesh_pub = rospy.Publisher('/mesh', Marker, queue_size=10)

        self.pub_curr_ee_pose_array = rospy.Publisher('/curr_ee_poses', PoseArray, queue_size=1)
        self.pub_ee_pose_array = rospy.Publisher('/ee_poses', PoseArray, queue_size=1)
        self.pub_mb_pose_array = rospy.Publisher('/mb_poses', PoseArray, queue_size=1)

        # self.ani = FuncAnimation(self.vis.fig, self.vis.update_plot, init_func=self.vis.plot_init)
        # boat_df["x"] += 5
        # boat_df["y"] += 5
        self.boat_df["visited"] = False
        
        self.boat_df = self.boat_df.sort_values(['x', 'z', 'y'], ascending=[True, True, True])
        self.boat_center_x, self.boat_center_y = self.boat_df[['x', 'y']].mean(axis=0)
        self.boat_length = self.boat_df["x"].max() - self.boat_df["x"].min()
        self.boat_width = self.boat_df["y"].max() - self.boat_df["y"].min()

        self.boat_xy_hull = ConvexHull(self.boat_df[["x", "y"]])
        self.boat_hull_vertices = self.boat_df.iloc[self.boat_xy_hull.vertices][["x", "y"]] #.sort_values(['x', 'y'], ascending=[True, True])
        self.boat_hull_vertices["angle_around_center"] = np.arctan2(self.boat_hull_vertices["y"] - self.boat_center_y, self.boat_hull_vertices["x"] - self.boat_center_x)
        self.boat_hull_vertices = self.boat_hull_vertices.sort_values(["angle_around_center"], ascending=[True])


        self.hull_pub = rospy.Publisher("convex_hull_marker", Marker, queue_size=10)
        self.boat_df["near_surface"] = ~check_point_in_hull(self.boat_df[["x", "y"]], self.boat_xy_hull, eps=-0.01)
        self.boat_df["angle_around_center"] = np.arctan2(self.boat_df["y"] - self.boat_center_y, self.boat_df["x"] - self.boat_center_x)
        self.boat_df = self.boat_df.sort_values(["angle_around_center"], ascending=[True])

        # print(os.getcwd())
        # time.sleep(2)
        # exit()
        
        # visualise original boat
        self.boat_facet_pose_array = process_pose_array(self.boat_df, dist = 0, horizontal=False, num_skip=1)
        self.boat_vis_pub = rospy.Publisher('/boat_facet_poses', PoseArray, queue_size=10)

        if len(self.boat_df) > 0:
            self.next_wp = 0
        else:
            rospy.loginfo("There are no points in the given boat.csv")
        
        self.arm_reach = 1
        self.mb_arm_base_height = 0.25

        # rospy.wait_for_service('wp_2_twist_srv')
        self.rate = rospy.Rate(0.1)  # 10 Hz



    def run(self):
        while not rospy.is_shutdown():
            # mb_unvisited_wp = self.boat_df.iloc[:1] #[self.boat_df["visited"] == False  & (self.boat_df["near_surface"] == True)]
            # call wp2twist service to get to boat_df.iloc[self.next_wp]
            # while(len(self.boat_df[self.boat_df["visited"] == True]) < 0.99 * len(self.boat_df)):
            while len(self.boat_df[(self.boat_df["near_surface"] == True)]) > self.next_wp:
                mb_unvisited_wp = self.boat_df[(self.boat_df["near_surface"] == True)].iloc[self.next_wp:self.next_wp + 1]
                print("================================")
                print(mb_unvisited_wp)
                print("================================")
                self.plot_convex_hull()
                # visualise original boat
                self.boat_vis_pub.publish(self.boat_facet_pose_array)
                self.mesh_pub.publish(self.mesh)

                try:
                    # rospy.loginfo(mb_unvisited_wp)
                    if len(mb_unvisited_wp) == 0: break
                    mb_unvisited_wp["z"] += self.mb_arm_base_height
                    self.mobile_base_pose_array = process_pose_array(mb_unvisited_wp, dist = 0.35*self.arm_reach, horizontal=True, arm_base_height=self.mb_arm_base_height)
                    base_position = self.mobile_base_pose_array.poses[0].position
                    base_x, base_y, base_z = base_position.x, base_position.y, base_position.z
                    base_point = np.array([base_x, base_y, base_z]).T
                    boat_point_dist = self.boat_df[["x", "y", "z"]][(self.boat_df["visited"] == False)].sub(base_point, axis=1).pow(2).sum(axis=1).pow(.5)
                    # boat_point_dist = self.boat_df[["x", "y", "z"]][(self.boat_df["visited"] == False)].sub(base_point, axis=1).pow(2).sum(axis=1).pow(.5)
                    self.pub_mb_pose_array.publish(self.mobile_base_pose_array)

                    # new_boat_df = self.boat_df.copy(deep=True)                    

                    # print(check_point_in_hull([[self.boat_center_x + self.boat_length/2, self.boat_center_y + self.boat_width/2]], self.boat_xy_hull, eps=0.2, print_eps=True))
                    # print(check_point_in_hull([[self.boat_center_x - self.boat_length/2, self.boat_center_y - self.boat_width/2]], self.boat_xy_hull, eps=0.2, print_eps=True))
                    # print(check_point_in_hull([[self.boat_center_x - self.boat_length/2, self.boat_center_y + self.boat_width/2]], self.boat_xy_hull, eps=0.2, print_eps=True))
                    # print(check_point_in_hull([[self.boat_center_x + self.boat_length/2, self.boat_center_y - self.boat_width/2]], self.boat_xy_hull, eps=0.2, print_eps=True))

                    print([base_x, base_y], check_point_in_hull([[base_x, base_y]], self.boat_xy_hull, eps=0.2, print_eps=True))
                    if not check_point_in_hull([[base_x, base_y]], self.boat_xy_hull, eps=0.2, print_eps=True):
                        
                        # Call movebase service
                        call_movebase_service(self.mobile_base_pose_array.poses[0])


                        unvisited_reachable_poses = self.boat_df[(self.boat_df["visited"] == False) & (boat_point_dist < 0.75*self.arm_reach)]
                        # print("unvisited_reachable_poses : ", len(unvisited_reachable_poses))
                        print(unvisited_reachable_poses)
                        # unvisited_reachable_poses = all_reachable_poses[all_reachable_poses["visited"] == False]
                        farthest_point_idx = boat_point_dist[unvisited_reachable_poses.index].idxmax()
                        
                        pose_visit_order = create_graph_perform_dfs(self.boat_df.loc[unvisited_reachable_poses.index, :], self.boat_df, base_point, farthest_point_idx)
                        for node_idx in pose_visit_order:
                            self.boat_vis_pub.publish(self.boat_facet_pose_array)
                            self.mesh_pub.publish(self.mesh)
                            self.pub_mb_pose_array.publish(self.mobile_base_pose_array)

                            self.end_effector_pose_array = process_pose_array(self.boat_df.loc[unvisited_reachable_poses.index, :], #.iloc[node_idx:node_idx+1], 
                                                                            dist = 0.3*self.arm_reach, horizontal=False, num_skip=1)
                            self.pub_curr_ee_pose_array.publish(self.end_effector_pose_array)
                            
                            pose_to_visit = process_pose_array((self.boat_df.loc[node_idx:node_idx+1,:]), 
                                                        dist = 0.3*self.arm_reach, horizontal=False, num_skip=1)
                            
                            # Call Manipulator service
                            call_manipulator_service(pose_to_visit.poses[0])

                            self.boat_df.loc[node_idx, ["visited"]] = True
                            self.visited_end_effector_pose_array = process_pose_array(self.boat_df[self.boat_df["visited"] == True], 
                                                                                dist = 0.3*self.arm_reach, horizontal=False, num_skip=1)
                            self.pub_ee_pose_array.publish(self.visited_end_effector_pose_array)
                            print(" ", end="")
                        
                    
                    # rospy.loginfo(boat_point_dist.idxmax())

                    # mb_unvisited_wp = self.boat_df.iloc[boat_point_dist.sample(1).index]
                    # nearest_surface_point_idx = boat_point_dist[self.boat_df[(self.boat_df["near_surface"]==True)&(self.boat_df["visited"] == False)].index].idxmin()
                    # mb_unvisited_wp = self.boat_df.iloc[farthest_visited_point_idx:farthest_visited_point_idx+1]
                    # boat_hull_point_dist = self.boat_hull_vertices.sub(farthest_visited_point_idx, axis=1).pow(2).sum(axis=1).pow(.5)

                    self.next_wp += 3

                    # mb_unvisited_wp = self.boat_df.iloc[nearest_surface_point_idx:nearest_surface_point_idx+1]
                    # rospy.loginfo("Going to the next waypoint: ", pose)

                    time.sleep(1)

                    # push_msg = rospy.ServiceProxy('wp_2_twist_srv', GoToWP)
                    # response = push_msg(pose)
                    # rospy.loginfo(response)
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: %s"%e)
                
                # # select points reachable by arm in a strip ==> self.next_wp is first unvisited point (because sorted)
                # self.next_wp +=50
                # self.rate.sleep()
            self.boat_vis_pub.publish(self.boat_facet_pose_array)
            self.mesh_pub.publish(self.mesh)
            self.pub_ee_pose_array.publish(self.visited_end_effector_pose_array)

    def plot_convex_hull(self):
        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = "map"  # Set the appropriate frame ID
        marker.header.stamp = rospy.Time.now()
        marker.ns = "convex_hull"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Set the line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0

        # Add the convex hull vertices as points to the marker
        for i in range(len(self.boat_hull_vertices)):
            point = Point()
            point.x = self.boat_hull_vertices["x"].iloc[i]
            point.y = self.boat_hull_vertices["y"].iloc[i]
            point.z = 0.0
            marker.points.append(point)

        # Close the polygon by adding the first vertex again
        point = Point()
        point.x = self.boat_hull_vertices["x"].iloc[0]
        point.y = self.boat_hull_vertices["y"].iloc[0]
        point.z = 0.0
        marker.points.append(point)

        # Create a publisher for the marker

        # Publish the marker
        self.hull_pub.publish(marker)
            

if __name__ == '__main__':
    try:
        bt = BehaviourTracker()
        bt.run()
    except rospy.ROSInterruptException:
        pass

