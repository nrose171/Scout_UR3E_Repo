Dependencies:
[Go Into Sub Directories to see how to install dependencies of each] \
ROS Noetic \
UGV_SIM \
Scout_Gazebo \
Industrial Robotics UR \
ros navigation stack \
gmapping \
ros moveit \
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

To start gazebo world: \
roslaunch scout_ure scout_ure_empty_world.launch

To run navigation and moveit: \
roslaunch scout_ure scout_ure_proj.launch

To run behavior tracker: \
roslaunch coverage_wp_planner behavior_tracker.launch 
