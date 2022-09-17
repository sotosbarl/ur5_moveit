#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
import copy
from math import atan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("right_arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")

hand_group.set_named_target("open")
plan2 = hand_group.go()

# Put the arm in the start position
arm_group.set_named_target("test")
plan1 = arm_group.go()
print(arm_group.get_current_pose().pose)

# put the arm at the 1st grasping position
pose_target = geometry_msgs.msg.Pose()
roll, pitch, yaw = 3.14 , 0 , -1.57
quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
#type(pose) = geometry_msgs.msg.Pose
pose_target.orientation.x = quaternion[0]
pose_target.orientation.y = quaternion[1]
pose_target.orientation.z = quaternion[2]
pose_target.orientation.w = quaternion[3]

pose_target.position.x = -0.5
pose_target.position.y = 0.17
pose_target.position.z = 1.09
arm_group.set_pose_target(pose_target)
# arm_group.setPlanningTime(10)
plan1 = arm_group.plan()


plan1 = arm_group.go()
#print(arm_group.get_current_pose().pose)

rospy.sleep(5)


#move a bit closer

pose_target.position.x = -0.5
pose_target.position.y = 0.01
pose_target.position.z = 1.06
arm_group.set_pose_target(pose_target)
# arm_group.setPlanningTime(10)
plan1 = arm_group.plan()


plan1 = arm_group.go()


rospy.sleep(5)

# arm_group.execute(plan3)


# plan = arm_group.plan()
# if not plan.joint_trajectory.points:
#     print('valid point')
# plan1 = arm_group.go()


# close the gripper
hand_group.set_named_target("close")
plan2 = hand_group.go()
rospy.sleep(7)

#lift the object
pose_target.position.x = -0.5
pose_target.position.y = -0
pose_target.position.z = 1.2
arm_group.set_pose_target(pose_target)
# arm_group.setPlanningTime(10)
plan1 = arm_group.plan()
plan1 = arm_group.go()



#move the object to the goal position
pose_target.position.x = 0.2
pose_target.position.y = -0.2
pose_target.position.z = 1.06
arm_group.set_pose_target(pose_target)
# arm_group.setPlanningTime(10)
plan1 = arm_group.plan()
plan1 = arm_group.go()


hand_group.set_named_target("open")
plan2 = hand_group.go()

#move away
pose_target.position.x = 0.2
pose_target.position.y = -0.2
pose_target.position.z = 1.25
arm_group.set_pose_target(pose_target)
# arm_group.setPlanningTime(10)
plan1 = arm_group.plan()
plan1 = arm_group.go()
