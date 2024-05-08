Dependencies:
[Go Into Sub Directories to see how to install dependencies of each] \
ROS Noetic \
UGV_SIM \
Scout_Gazebo \
Industrial Robotics UR \
move_base \
object_spawner 

To Build: \
mkdir -p catkin_ws/src \
cd catkin_ws/src \
git clone https://github.com/nrose171/Scout_UR3E_Repo.git \
cd .. \
catkin build 

To source: \
cd catkin_ws \
source devel/setup.bash 

To run navigation: \
roslaunch scout_ur3e scout_ur3e_navigation.launch 

To run moveit: \
roslaunch scout_ur3e scout_ur3e_manip.launch Use_RVIZ:=true 

To run behavior tracker: \
roslaunch coverage_wp_planner waypoint2twist.launch 
