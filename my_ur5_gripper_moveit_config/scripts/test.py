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
scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("right_arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")

rospy.sleep(2)

# p = geometry_msgs.msg.PoseStamped()
# p.header.frame_id = robot.get_planning_frame()
# p.pose.position.x = 0.
# p.pose.position.y = -0.4
# p.pose.position.z = 1.14
# scene.add_box("table", p, (0.07, 0.67, 0.5))

arm_group.set_named_target("test")
plan1 = arm_group.go()
rospy.sleep(5)
arm_group.set_named_target("test2")
plan1 = arm_group.go()
rospy.sleep(5)
arm_group.set_named_target("test3")
plan1 = arm_group.go()
rospy.sleep(5)

#
# pose_target = geometry_msgs.msg.Pose()
#
# pose_target.position.x = 0.5
# pose_target.position.y = -0
# pose_target.position.z = 1.09
#
# roll, pitch, yaw = 3.14 , 0 , -1.57 #atan(pose_target.position.y / pose_target.position.x)
# quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
# #type(pose) = geometry_msgs.msg.Pose
# pose_target.orientation.x = quaternion[0]
# pose_target.orientation.y = quaternion[1]
# pose_target.orientation.z = quaternion[2]
# pose_target.orientation.w = quaternion[3]
#
# arm_group.set_pose_target(pose_target)
# arm_group.set_planning_time(15)
# plan1 = arm_group.plan()
#
# # arm_group.set_named_target("test")
# plan1 = arm_group.go()







hand_group.set_named_target("open")
plan2 = hand_group.go()
