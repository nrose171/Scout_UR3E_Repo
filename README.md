To Build:
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/nrose171/Scout_UR3E_Repo/new/scout_planner?filename=README.md
cd ..
catkin build

To source:
cd catkin_ws
source devel/setup.bash

To run navigation:
roslaunch scout_ur3e scout_ur3e_navigation.launch

To run moveit:
roslaunch
