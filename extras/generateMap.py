#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from nav_msgs.srv import GetMap

def generate_map():
    rospy.init_node('generate_map')

    rospy.wait_for_service('static_map')
    try:
        get_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_map()
        
        width = response.map.info.width
        height = response.map.info.height
        resolution = response.map.info.resolution

        map_data = np.array(response.map.data).reshape((height, width))
        map_data[map_data == -1] = 205
        map_data[map_data == 0] = 255
        map_data[map_data == 100] = 0

        map_img = cv2.flip(map_data, 0)
        cv2.imwrite('occupancy_map.png', map_img)
        rospy.loginfo("Map saved as occupancy_map.png")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        generate_map()
    except rospy.ROSInterruptException:
        pass
