import rospy
import numpy as np
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

def get_normals(csv_file):
    pose_arr = PoseArray()
    pose_arr.header.frame_id = 'map'
    pose_arr.header.stamp = rospy.Time.now()

    pose_arr_2 = PoseArray()
    pose_arr_2.header.frame_id = 'map'
    pose_arr_2.header.stamp = rospy.Time.now()

    centers = []
    quat = []
    centers_2 = []
    quat_2 = []
    with open(csv_file) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            if row[0] == 'x':
                continue
            # print(row)
            # print(row[0])
            row = np.array(row, dtype=np.float64)
            center = np.zeros((3,),dtype=np.float64)
            q = np.zeros((4,),dtype=np.float64)
            center_2 = np.zeros((3,),dtype=np.float64)
            q_2 = np.zeros((4,),dtype=np.float64)
            center[0] = float(row[0])
            center[1] = float(row[1])
            center[2] = float(row[2])
            q[0] = float(row[3])
            q[1] = float(row[4])
            q[2] = float(row[5])
            q[3] = float(row[6])
            
            dist = 0.25
            conj = [-1.0*q[0], -1.0*q[1], -1.0*q[2], q[3]]
            mag = math.sqrt(math.pow(q[0], 2)+math.pow(q[1], 2)+math.pow(q[2], 2)+math.pow(q[3], 2))

            q_2[0] = conj[0]/math.pow(mag, 2)
            q_2[1] = conj[1]/math.pow(mag, 2)
            q_2[2] = conj[2]/math.pow(mag, 2)
            q_2[3] = conj[3]/math.pow(mag, 2)

            # D_x = 2 * (q[0]*q[2] + q[3]*q[1])
            # D_y = 2 * (q[1]*q[2] - q[3]*q[0])
            # D_z = 1 - 2 * (q[0]*q[0] + q[1]*q[1])

            D_x = 2 * (q[1]*q[3] + q[0]*q[2])
            D_y = 2 * (q[2]*q[3] - q[0]*q[1])
            D_z = 1 - 2 * (q[1]*q[1] + q[2]*q[2])

            D_arr = [D_x, D_y, D_z]
            D = [float(i)/sum(D_arr) for i in D_arr]

            # center_2[0] = float(row[0]) + dist * D[0]
            # center_2[1] = float(row[1]) + dist * D[1]
            # center_2[2] = float(row[2]) + dist * D[2]
            center_2[0] = float(row[0]) 
            center_2[1] = float(row[1]) 
            center_2[2] = float(row[2]) 
            centers.append(center)
            quat.append(q)
            centers_2.append(center_2)
            quat_2.append(q_2)
    centers = np.array(centers)
    quat = np.array(quat)
    centers_2 = np.array(centers_2)
    quat_2 = np.array(quat_2)
    # print("centers max = {}".format(np.amax(centers,axis=0)))
    # print("centers min = {}".format(np.amin(centers,axis=0)))
    for i in range(len(centers)):
        pose = Pose()
        pose.position.x = centers[i,0]
        pose.position.y = centers[i,1]
        pose.position.z = centers[i,2]
        pose.orientation.x = quat[i,0]
        pose.orientation.y = quat[i,1]
        pose.orientation.z = quat[i,2]
        pose.orientation.w = quat[i,3]
        pose_arr.poses.append(pose)

    for i in range(len(centers_2)):
        pose = Pose()
        pose.position.x = centers_2[i,0]
        pose.position.y = centers_2[i,1]
        pose.position.z = centers_2[i,2]
        pose.orientation.x = quat_2[i,0]
        pose.orientation.y = quat_2[i,1]
        pose.orientation.z = quat_2[i,2]
        pose.orientation.w = quat_2[i,3]
        pose_arr_2.poses.append(pose)


    return pose_arr, pose_arr_2

def main():
    rospy.init_node('publish_normal')
    parser = argparse.ArgumentParser(description='Publish normal vectors as quaternions for visualization in RViz')
    parser.add_argument('-c','--csv_file', type=str, help='csv file containing normal vectors', default='boat.csv')
    parser.add_argument('-m','--mesh_file', type=str, help='mesh file to visualize normals')
    args = parser.parse_args()

    pub = rospy.Publisher('/pose', PoseArray, queue_size=10)
    pub_2 = rospy.Publisher('/pose_2', PoseArray, queue_size=10)
    mesh_pub = rospy.Publisher('/mesh', Marker, queue_size=10)
    rate = rospy.Rate(1)
    poses, poses_2 = get_normals(args.csv_file)
    mesh = get_mesh('../models/boat/meshes/boat.dae')
    print("publishing normals as posearray")
    while not rospy.is_shutdown():
        mesh_pub.publish(mesh)
        pub.publish(poses)
        pub_2.publish(poses_2)
        rate.sleep()

if __name__ == '__main__':
    main()
