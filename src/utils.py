#!/usr/bin/env python3
import numpy as np
from math import radians, sin, cos, atan2, sqrt, pi as PI, degrees
from geometry_msgs.msg import Quaternion

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :returns a geometry_msgs/Quaternion object populated with qx, qy, qz, qw
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return Quaternion(qx, qy, qz, qw)

# vector giving direction of camera face wrt camera base [-1 0 0]
# vector giving target direction of the [camera target] - [camera_base]
# required transform will be a combination of both
# known: altitude(alt), angle of approach from vertical(theta), target_vector
# [camera_vector] = [target_vector] + [alt * sin(theta), alt * sin(theta), alt]

### NEW APPROACH: ALIGN ANGLES ABOUT ALL 3 AXIS ###
# Position vector (global reference frame) [x1, y1, z1] | Magnitude of vector = M1
# Camera direction vector(local frame) [x2, y2, z2] | Magnitude of vector = M2
# angles of position vector with x, y, z axes = [arccos(x1)/M1, arccos(y1)/M1, arccos(z1)/M1]
# angles of camera vector with x, y, z axes = [arccos(x2)/M1, arccos(y2)/M1, arccos(z2)/M2]

def get_camera_rpy(x, y, z):
  # Known camera parameter: Facing along local frame x axis
  # x = x * 0.9
  # y = y * 0.9
  # z = z * 1.25
  origin_x_offset = 3.0
  origin_y_offset = 3.0
  sign_x = 0.0
  sign_y = 0.0

  if(x > 0.0):
    sign_x = -1
  elif(x < 0.0):
    sign_x = +1
  else:
    sign_x = 0.0

  if(y > 0.0):
    sign_y = -1
  elif(y < 0.0):
    sign_y = +1
  else:
    sign_y = 0.0

  x += sign_x * origin_x_offset
  y += sign_y * origin_y_offset

  roll = 0.0
  pitch = (PI/2 - atan2(z, sqrt(x**2 + y**2)))
  yaw = (atan2(y, x) - PI)
  return [roll, pitch, yaw]

# vector along 

def input_mode():
  global x, y, z
  takeInput = True
  while takeInput:
    user_input = input("> Command: ")
    if('exit' in user_input):
      print("Exiting.")
      takeInput = False
      break
    else:
      x = float(str(user_input[0]) + str(user_input[1]) + str(user_input[2]) + str(user_input[3]) + str(user_input[4]))
      y = float(str(user_input[6]) + str(user_input[7]) + str(user_input[8]) + str(user_input[9]) + str(user_input[10]))
      z = float(str(user_input[12]) + str(user_input[13]) + str(user_input[14]) + str(user_input[15]) + str(user_input[16]))
      print("Input received: ", x, y, z)
      rpy = get_camera_rpy(x, y, z)
      print("RPY in radians: ", rpy)
      print("RPY in degrees: ", (degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2])))
      # print(90 - atan2(z, sqrt(x**2 + y**2)) * PI / 180)

if __name__ == '__main__':
  x = sqrt(2)
  y = sqrt(2)
  z = 2
  input_mode()
  # rpy = get_camera_rpy(x, y, z)
  # print(rpy)
  # print(degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2]))
  # print(90 - atan2(z, sqrt(x**2 + y**2)) * PI / 180)

