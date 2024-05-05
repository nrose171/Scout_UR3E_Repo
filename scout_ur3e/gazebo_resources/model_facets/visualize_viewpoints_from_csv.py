#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import pandas as pd
import transforms3d as t3d
import os
from geometry_msgs.msg import Pose, PoseArray
import argparse
import csv
from visualization_msgs.msg import Marker
import math

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

def read_csv_get_poses_arr(csv_file):
    df = pd.read_csv(csv_file)
    points = df.iloc[:, :3].values
    quaternions = df.iloc[:, 3:7].values

    pose_arr_1 = PoseArray()
    pose_arr_1.header.frame_id = 'map'
    pose_arr_1.header.stamp = rospy.Time.now()

    for i in range(len(points)):
        pose = Pose()
        pose.position.x = points[i, 0]
        pose.position.y = points[i, 1]
        pose.position.z = points[i, 2]
        pose.orientation.x = quaternions[i, 0]
        pose.orientation.y = quaternions[i, 1]
        pose.orientation.z = quaternions[i, 2]
        pose.orientation.w = quaternions[i, 3]
        pose_arr_1.poses.append(pose)

    return points, quaternions, pose_arr_1

def process_pose_array(points, quaternions):
    pose_arr_2 = PoseArray()
    pose_arr_2.header.frame_id = 'map'
    pose_arr_2.header.stamp = rospy.Time.now()

    dist = .750  # Distance to shift the points (in meters)
    flip_matrix = np.array([
        [-1, 0, 0],
        [0, 1, 0],
        [0, 0, -1]
    ])

    for i in range(len(points)):
        # Convert quaternion to rotation matrix
        q = quaternions[i]
        rot_matrix = np.array([
            [1 - 2*q[1]**2 - 2*q[2]**2, 2*q[0]*q[1] - 2*q[2]*q[3], 2*q[0]*q[2] + 2*q[1]*q[3]],
            [2*q[0]*q[1] + 2*q[2]*q[3], 1 - 2*q[0]**2 - 2*q[2]**2, 2*q[1]*q[2] - 2*q[0]*q[3]],
            [2*q[0]*q[2] - 2*q[1]*q[3], 2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[0]**2 - 2*q[1]**2]
        ])

        # Calculate the direction vector from the rotation matrix
        direction = rot_matrix[:, 0]  # First column of the rotation matrix

        # Shift the point by the specified distance in the direction of the quaternion
        shifted_point = points[i] + dist * direction
        # direction = t3d.mat2quat(rot_matrix[:, 0].T)

                # Multiply the rotation matrix by the flip matrix
        flipped_matrix = np.dot(rot_matrix, flip_matrix)

        # Convert the flipped rotation matrix back to a quaternion
        flipped_quat = quaternion_from_matrix(flipped_matrix)

        pose = Pose()
        pose.position.x = shifted_point[0]
        pose.position.y = shifted_point[1]
        pose.position.z = shifted_point[2]
        pose.orientation.x = flipped_quat[0]
        pose.orientation.y = flipped_quat[1]
        pose.orientation.z = flipped_quat[2]
        pose.orientation.w = flipped_quat[3]
        pose_arr_2.poses.append(pose)

    return pose_arr_2

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

def main():
    rospy.init_node('publish_normal')
    rospack = rospkg.RosPack()
    dir_path = rospack.get_path('object_spawner')

    parser = argparse.ArgumentParser(description='Publish normal vectors as quaternions for visualization in RViz')
    parser.add_argument('-c','--csv_file', type=str, help='csv file containing normal vectors', default=dir_path + "/gazebo_resources/model_facets/boat.csv")
    parser.add_argument('-m','--mesh_file', type=str, help='mesh file to visualize normals', default= dir_path + "/gaebo_resources/models/boat/meshes/boat.dae")
    args = parser.parse_args()

    pub = rospy.Publisher('/pose', PoseArray, queue_size=10)
    pub_2 = rospy.Publisher('/pose_2', PoseArray, queue_size=10)
    mesh_pub = rospy.Publisher('/mesh', Marker, queue_size=10)
    rate = rospy.Rate(1)

    points, quaternions, poses = read_csv_get_poses_arr(args.csv_file)
    poses_2 = process_pose_array(points, quaternions)
    mesh = get_mesh(args.mesh_file)
    print("publishing normals as posearray")
    while not rospy.is_shutdown():
        mesh_pub.publish(mesh)
        pub.publish(poses)
        pub_2.publish(poses_2)
        rate.sleep()

if __name__ == '__main__':
    main()