#!/usr/bin/env python

import rospy
import csv
from gazebo_msgs.msg import ModelStates

def callback(data):
    # Find the index of the 'turtlebot3' model
    if 'husky' in data.name:
        index = data.name.index('husky')
        pose = data.pose[index]

        # Write the pose to a CSV file
        with open('husky_groundtruth.csv', mode='a') as file:
            writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow([rospy.get_time(), pose.position.x, pose.position.y, pose.position.z,
                             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

def listener():
    rospy.init_node('turtlebot3_pose_listener', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.spin()

if __name__ == '__main__':
    # Initialize the CSV file with headers
    with open('husky_groundtruth.csv', mode='w') as file:
        writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['time', 'position_x', 'position_y', 'position_z',
                         'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'])

    listener()