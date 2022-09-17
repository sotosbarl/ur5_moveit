# ur5_moveit
Open the workspace which containts robot description packages (i.e. the X_ROS folder).

Follow the instructions here to install gazebo_pkgs: https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation

open ~/{your_workspace_name}/src/X_ROS/src/X_UR/ee_ur5e/ur5 and delete the ee_ur5_gazebo folder (package)

Open {your_workspace_name)/src

```git clone https://github.com/sotosbarl/ur5_moveit.git```

```cd ..  ``` (to go in the previous folder)

```catkin_make``` (to compile the workspace, it should reach 100% without errors, if errors occur contact with me please)

```roslaunch ee_ur5e_gazebo ee_ur5e_bringup.launch``` (for the simulation to begin)

(It starts in paused mode. Press the play button on the bottom) After that:

In another terminal: ```roslaunch my_ur5_gripper_moveit_config demo_gazebo_mine.launch``` (for the moveit node to start, rviz also starts and we can send commands to the robot from there but we will use a script.) 

So run the script. In another terminal: ```rosrun my_ur5_gripper_moveit_config pick_n_place.py```


