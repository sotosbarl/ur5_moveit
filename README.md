# ur5_moveit
You already completed the following steps: 

* * Open the workspace which containts robot description packages (i.e. the X_ROS folder). * *

* * Follow the instructions here to install gazebo_pkgs: https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation * *

* * open ~/{your_workspace_name}/src/X_ROS/src/X_UR/ee_ur5e/ur5 and delete the ee_ur5_gazebo folder (package) * *


So proceed. Delete the packages  my_ur5_gripper_moveit_config and ee_ur5e_gazebo because you will install the new versions.

Open {your_workspace_name)/src

```git clone https://github.com/sotosbarl/ur5_moveit.git```

```cd ..  ``` (to go in the previous folder)

```catkin_make``` (to compile the workspace, it should reach 100% without errors, if errors occur contact with me please)

You are done with installation. You can run the simulation.

```roslaunch ee_ur5e_gazebo ee_ur5e_bringup.launch``` (for the simulation to begin)

(It starts in paused mode. Press the play button on the bottom) After that:

In another terminal,  run the script: ```rosrun opencv_services object_detector.py```


