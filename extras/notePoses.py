#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped

ground_truth_pose = None

def ground_truth_callback(model_states_msg):
    global ground_truth_pose
    # Find the index of the Husky robot in the model states
    # i = model_states_msg.name.index("Apriltag36_11_00005")
    # print("huskypos: ", model_states_msg.pose[i].position.x,model_states_msg.pose[i].position.y,model_states_msg.pose[i].position.z)
    # print("orientation: ", model_states_msg.pose[i].orientation.x,model_states_msg.pose[i].orientation.y,model_states_msg.pose[i].orientation.z,model_states_msg.pose[i].orientation.w)

    for i in range(len(model_states_msg.name)):
        print("Marker: ", model_states_msg.name[i][-2:])
        print("position: ", model_states_msg.pose[i].position.x,model_states_msg.pose[i].position.y,model_states_msg.pose[i].position.z)
        print("orientation: ", model_states_msg.pose[i].orientation.x,model_states_msg.pose[i].orientation.y,model_states_msg.pose[i].orientation.z,model_states_msg.pose[i].orientation.w)
        print("-------------------------------------------------")
    exit()
   
def main():
    rospy.init_node('sync_poses', anonymous=True)

    # Subscribers for ground truth and estimated poses
    ground_truth_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, ground_truth_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
