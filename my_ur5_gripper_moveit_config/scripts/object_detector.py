#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
import copy
from math import atan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from opencv_services.srv import *
# from opencv_services.srv import box_and_target_position

def robot_client(position):
    print('dsdsds')
    print(position)



# init a node as usual
rospy.init_node('opencv_client')

# wait for this sevice to be running
rospy.wait_for_service('TargetPosition')

# Create the connection to the service.
server_call = rospy.ServiceProxy('TargetPosition' , box_and_target_position)

# Create an object of the type TriggerRequest.
target = box_and_target_position()

# Now send the request through the connection
result = server_call(target)

robot_client(result)
