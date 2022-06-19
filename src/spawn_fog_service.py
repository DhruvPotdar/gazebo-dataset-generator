#!/usr/bin/env python
import time
import rospy

from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from gazebo_msgs.srv import SetLightProperties, SetLightPropertiesRequest
from geometry_msgs.msg import Pose, Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64    
from std_msgs.msg import ColorRGBA  
from std_srvs.srv import Empty 

service_name = '/gazebo/set_light_properties'
rospy.loginfo("Waiting for service " + str(service_name))
rospy.wait_for_service(service_name)
rospy.loginfo("Service Found "+str(service_name))

set_light = rospy.ServiceProxy(service_name, SetLightProperties)

light_name = 'sun'
cast_shadows = True

difuse = ColorRGBA()
difuse.r = float(0/255)
difuse.g = float(0/255)
difuse.b = float(0/255)
difuse.a = float(0/255)

specular = ColorRGBA()
specular.r = float(0/255)
specular.g = float(0/255)
specular.b = float(0/255)
specular.a = float(0/255)

attenuation_constant = 1.0
attenuation_linear = 1.0
attenuation_qudratic = 1    

direction = Vector3()
direction.x = -0.483368
direction.y = 0.096674
direction.z = -0.870063

pose = Pose()
pose.position.x = 0.00
pose.position.y = 0.00
pose.position.z = 10.00

pose.orientation.x = 0.00
pose.orientation.y = 0.00
pose.orientation.z = 0.00
pose.orientation.w = 1.00

response = set_light(light_name, cast_shadows, difuse, specular, attenuation_constant, attenuation_linear, attenuation_qudratic, direction, pose)

print(response)