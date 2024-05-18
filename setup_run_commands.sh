# :system commands:
sudo apt-get update
sudo apt-get install git python3-catkin-tools ros-noetic-gazebo-msgs \
ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-dev \
ros-noetic-control-toolbox ros-noetic-controller-interface \
ros-noetic-moveit ros-noetic-moveit-core ros-noetic-gazebo-ros-control \
ros-noetic-moveit-kinematics ros-noetic-velocity-controllers \
ros-noetic*joint-trajectory-controller* ros-noetic-effort-controllers \
ros-noetic-trac-ik-kinematics-plugin ros-$ROS_DISTRO-universal-robots \
ros-noetic-joint-state-controller ros-noetic-tf2-sensor-msgs python3-pip
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

pip install pandas scipy networkx

# # :source ros and repo in .bashrc:
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /root/Scout_UR3E_Repo/devel/setup.bash" >> ~/.bashrc


# roslaunch scout_gazebo_sim scout_ur3e_empty_world.launch
# roslaunch coverage_wp_planner waypoint2twist.launch