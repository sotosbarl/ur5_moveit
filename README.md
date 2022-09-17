# ur5_moveit
Open the workspace which containts robot description packages (i.e. the X_ROS folder).

Open {your_workspace_name)/src

git clone 

roslaunch ee_ur5e_gazebo ee_ur5e_bringup.launch (for the simulation to begin)

(It starts in paused mode. Press the play button on the bottom) After that:

roslaunch my_ur5_gripper_moveit_config demo_gazebo_mine.launch (for the moveit node to start, rviz also starts and we can send commands to the robot from there but we will use a script.) So run the script.

rosrun my_ur5_gripper_moveit_config pick_n_place.py


