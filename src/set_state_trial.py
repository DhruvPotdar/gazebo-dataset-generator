#! /usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateResponse
from math import pi as PI, tan
 
from utils import get_quaternion_from_euler

### Face down orientation of camera ###
# roll 0 (GIMBAL LOCK!!)
# pitch pi/2
# yaw 0
# i.e. camera is along x axis
# Point that it points to is at the center of the top of the frame captured

GAZEBO_SET_MODEL_STATE_TOPIC = '/gazebo/set_model_state'
GAZEBO_CAMERA_MODEL_NAME = 'camera'
GAZEBO_REFERENCE_FRAME = 'world'

# Define origin of the center of the target object
TARGET_OBJECT_X = 0.0
TARGET_OBJECT_Y = 0.0
TARGET_OBJECT_Z = 0.0

rospy.init_node('camera_state_publisher')
rate = rospy.Rate(0.1)

ms = ModelState()
# Static state parameters
ms.model_name = GAZEBO_CAMERA_MODEL_NAME
ms.reference_frame = GAZEBO_REFERENCE_FRAME
ms.twist = Twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# Default pose
ms.pose = Pose(0.0, 0.0, 10.0)
ms.pose.orientation = get_quaternion_from_euler(0, PI/2, 0)

def set_state_service_call(model_state):
    rospy.wait_for_service(GAZEBO_SET_MODEL_STATE_TOPIC)
    service_response = SetModelStateResponse()
    try:
        set_state_service = rospy.ServiceProxy(GAZEBO_SET_MODEL_STATE_TOPIC, SetModelState)
        service_response = set_state_service(model_state)
        rospy.loginfo(service_response.status_message)
        if(service_response.success):
            pass
            # increment

        # return service_response
    except Exception as e:
        print(e)
    else:
        return service_response

if __name__ == '__main__':
    
    altitudes = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0]
    angles = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]

    for alt in altitudes:
        for theta in angles:
            x = alt * tan(theta) + TARGET_OBJECT_X
            y = x + TARGET_OBJECT_Y
            z = alt + TARGET_OBJECT_Z
            ms.pose.position.x = x
            ms.pose.position.y = y
            ms.pose.position.z = z

            set_state_service_call(ms)
            rate.sleep()

    rospy.loginfo('Execution complete. Shutting down node.')
    # rospy.spin()