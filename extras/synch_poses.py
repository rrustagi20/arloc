#!/usr/bin/env python
import pandas as pd
import rospy
import message_filters
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped

ground_truth_pose = None
amcl_pose = None

# Create an empty DataFrame with specified columns
df_columns = ["time", "position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"]
df_amcl = pd.DataFrame(columns=df_columns)
df_gt = pd.DataFrame(columns=df_columns)

def ground_truth_callback(model_states_msg):
    global ground_truth_pose
    # Find the index of the Husky robot in the model states
    husky_index = model_states_msg.name.index("husky")
    ground_truth_pose = model_states_msg.pose[husky_index]

    # rospy.loginfo("Ground Truth Pose: position (%f, %f, %f), orientation (%f, %f, %f, %f)",
    #               ground_truth_pose.position.x, ground_truth_pose.position.y, ground_truth_pose.position.z,
    #               ground_truth_pose.orientation.x, ground_truth_pose.orientation.y, ground_truth_pose.orientation.z, ground_truth_pose.orientation.w)

def amcl_pose_callback(amcl_pose_msg):
    amcl_pose = amcl_pose_msg.pose.pose

    # rospy.loginfo("AMCL Pose: position (%f, %f, %f), orientation (%f, %f, %f, %f)",
    #               amcl_pose.position.x, amcl_pose.position.y, amcl_pose.position.z,
    #               amcl_pose.orientation.x, amcl_pose.orientation.y, amcl_pose.orientation.z, amcl_pose.orientation.w)

    global df_amcl, df_gt
    time = rospy.Time.now().to_sec()
    # Create a dictionary with the amcl data
    data_amcl = {
        "time": time,
        "position_x": amcl_pose.position.x,"position_y": amcl_pose.position.y,"position_z": amcl_pose.position.z,
        "orientation_x": amcl_pose.orientation.x,"orientation_y": amcl_pose.orientation.y,"orientation_z": amcl_pose.orientation.z,"orientation_w": amcl_pose.orientation.w
    }
    # Append the new data to the DataFrame
    df_amcl = df_amcl.append(data_amcl, ignore_index=True)

    # Create a dictionary with the ground truth data
    data_gt = {
        "time": time,
        "position_x": ground_truth_pose.position.x,"position_y": ground_truth_pose.position.y,"position_z": ground_truth_pose.position.z,
        "orientation_x": ground_truth_pose.orientation.x,"orientation_y": ground_truth_pose.orientation.y,"orientation_z": ground_truth_pose.orientation.z,"orientation_w": ground_truth_pose.orientation.w
    }
    # Append the new data to the DataFrame
    df_gt = df_gt.append(data_gt, ignore_index=True)

def main():
    rospy.init_node('sync_poses', anonymous=True)

    # Subscribers for ground truth and estimated poses
    ground_truth_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, ground_truth_callback, queue_size=1)
    amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
    # Save the synchronized data to a new CSV file
    df_amcl.to_csv('office_vision_poses.csv', index=False)
    df_gt.to_csv('office_ground_vision_truth_poses.csv', index=False)
