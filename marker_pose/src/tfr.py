#!/usr/bin/python

import rospy
import math
# from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_multiply, quaternion_inverse
# import tf2_geometry_msgs  # For transforming geometry_msgs

# Define two quaternions
# quaternion1 = Quaternion(x=0.1, y=0.2, z=0.3, w=0.4)
quaternion1 = (0.0, 0.707106, 0.0, 0.707106)
quaternion2 = (0.0, 0.0, 0.0, 1.0)
# quaternion2 = (0.707106, 0.707106, 0.0, 0.0)

inv = quaternion_multiply(quaternion2,quaternion_inverse(quaternion1))  # Invert quaternion1
angle = math.acos(inv[0])*180/math.pi
print(inv)
print(angle)