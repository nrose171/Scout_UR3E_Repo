Dependencies:
<Go Into Sub Directories to see how to install dependencies of each>
ROS Noetic \n
UGV_SIM \n
Scout_Gazebo \n
Industrial Robotics UR \n
move_base \n
object_spawner \n

To Build: \n
mkdir -p catkin_ws/src \n
cd catkin_ws/src \n
git clone https://github.com/nrose171/Scout_UR3E_Repo.git \n
cd .. \n
catkin build \n

To source: \n
cd catkin_ws \n
source devel/setup.bash \n

To run navigation: \n
roslaunch scout_ur3e scout_ur3e_navigation.launch \n

To run moveit: \n
roslaunch scout_ur3e scout_ur3e_manip.launch Use_RVIZ:=true \n

To run behavior tracker: \n
roslaunch coverage_wp_planner waypoint2twist.launch \n
