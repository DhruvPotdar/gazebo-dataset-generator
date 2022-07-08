#! /usr/bin/env python3

from email.mime import base
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateResponse
from math import cos, pi as PI, sin, tan, radians
 
from utils import get_quaternion_from_euler, get_camera_rpy

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
rate = rospy.Rate(15)

ms = ModelState()
# Static state parameters
ms.model_name = GAZEBO_CAMERA_MODEL_NAME
ms.reference_frame = GAZEBO_REFERENCE_FRAME
ms.twist = Twist()
# Default pose
ms.pose = Pose()
ms.pose.position.x = 0.0
ms.pose.position.y = 0.0
ms.pose.position.z = 10.0
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
    
    print("Main called.")
    
    altitudes = [10.0, 20.0, 30.0, 40.0, 50.0]
    angles = [0.0, 45.0]
    base_yaw = []
    for yaw in range(0, 370, 10):
        base_yaw.append(float(yaw))
    print(base_yaw)
    for alt in altitudes:
        first_shot = True
        for theta in angles:
            if(theta == 0.0 and not first_shot):
                continue
                # base yaw is irrelevant for angle 0.0
            else:
                for base_y in base_yaw:           
                    r = alt * tan(radians(theta))
                    x = r * cos(radians(base_y)) + TARGET_OBJECT_X
                    y = r * sin(radians(base_y)) + TARGET_OBJECT_Y
                    z = alt + TARGET_OBJECT_Z
                    [roll, pitch, yaw] = get_camera_rpy(x, y, z)
                    ms.pose.orientation = get_quaternion_from_euler(roll, pitch, yaw)
                    ms.pose.position.x = x
                    ms.pose.position.y = y
                    ms.pose.position.z = z
                    print("Position: x: {} y: {} z: {} | Roll: {} Pitch: {} Yaw: {}".format(x, y, z, roll, pitch, yaw))
                    set_state_service_call(ms)
                    first_shot = False
                    rate.sleep()

    rospy.loginfo('Execution complete. Shutting down node.')
    # rospy.spin()