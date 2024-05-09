#! /usr/bin/env python3

from gazebo_msgs.srv import SpawnModel
import rospy
import rospkg
import geometry_msgs.msg as gm

rospack = rospkg.RosPack()
dir_path = rospack.get_path('scout_ur3e')
gz_re_path = dir_path + "/gazebo_resources/"

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

def spawn_model(m_path, m_name, start_pose):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_client(
        model_name=m_name,
        model_xml=open(m_path, 'r').read(),
        robot_namespace='/foo',
        initial_pose= start_pose,
        reference_frame='world'
    )

if __name__ == "__main__":
    # path = rospack.get_path('object_spawner') + '/models/arrow_red/model.sdf'
    # name = "arrow"
    # pose = gm.Pose(gm.Point(0, 0, 0.5), gm.Quaternion(0, 0, 0, 0))

    # for i in range(10):
    #     spawn_model(path, name + str(i), pose)

    boat_path = gz_re_path + "models/boat/model.sdf"
    boat_name = "boat"
    boat_pose = gm.Pose(gm.Point(0, 0, 0), gm.Quaternion(0, 0, 0, 0))
    spawn_model(boat_path, boat_name, boat_pose)