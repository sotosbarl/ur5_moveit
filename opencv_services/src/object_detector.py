#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
import copy
import opencv_services
from math import atan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from opencv_services.srv import TargetPosition


def robot_client(position):
    # print(type(position))
    x_target = position.box_position.x
    y_target = position.box_position.y
    z_target = position.box_position.z
    print(position)

    if y_target < 0.05 and z_target > 1:  #we care about things on the table

        # put the arm as close as possible to the object
        pose_target = geometry_msgs.msg.Pose()

        roll, pitch, yaw = 3.14 , 0 , -1.57
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]

        pose_target.position.x = x_target - 0.03
        pose_target.position.y = y_target + 0.3
        pose_target.position.z = z_target + 0.12

        arm_group.set_pose_target(pose_target)

        plan1 = arm_group.plan()
        plan1 = arm_group.go()

        rospy.sleep(5)

        pose_target.position.x = x_target - 0.03
        pose_target.position.y = y_target + 0.15
        pose_target.position.z = z_target

        arm_group.set_pose_target(pose_target)

        plan1 = arm_group.plan()
        plan1 = arm_group.go()
        rospy.sleep(5)

        #grab the object
        hand_group.set_named_target("close")
        plan2 = hand_group.go()
        rospy.sleep(7)

            #lift the object
        pose_target.position.x = x_target - 0.03
        pose_target.position.y = y_target + 0.17
        pose_target.position.z = z_target + 0.13
        arm_group.set_pose_target(pose_target)
        # arm_group.setPlanningTime(10)
        plan1 = arm_group.plan()
        plan1 = arm_group.go()



        #move the object to the goal position
        pose_target.position.x = -x_target
        pose_target.position.y = y_target + 0.17
        pose_target.position.z = z_target

        arm_group.set_pose_target(pose_target)
        # arm_group.setPlanningTime(10)
        plan1 = arm_group.plan()
        plan1 = arm_group.go()


        hand_group.set_named_target("open")
        plan2 = hand_group.go()

        #move away
        pose_target.position.x = -x_target
        pose_target.position.y = y_target
        pose_target.position.z = 1.25
        arm_group.set_pose_target(pose_target)
        # arm_group.setPlanningTime(10)
        plan1 = arm_group.plan()
        plan1 = arm_group.go()

        #return to the observation position
        arm_group.set_named_target("test")
        plan1 = arm_group.go()


# init a node as usual
rospy.init_node('opencv_client')

#initialize moveit commander
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("right_arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")


#Go to the observation position (above table)
arm_group.set_named_target("test")
plan1 = arm_group.go()

rospy.sleep(5)

# while True:
# wait for this sevice to be running
rospy.wait_for_service('TargetPosition')

# Create the connection to the service.
server_call = rospy.ServiceProxy('TargetPosition', TargetPosition)

# Create an object of the type TriggerRequest.
# target = TargetPosition()

# Now send the request through the connection
result = server_call()

robot_client(result)
