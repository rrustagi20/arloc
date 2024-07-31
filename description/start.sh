export HUSKY_LMS1XX_ENABLED=1
export HUSKY_REALSENSE_ENABLED=1
# source /home/rahul/ros/arloc/src/arloc/description/param_office.sh

# For mapping

# gnome-terminal -- roslaunch description office.launch
# gnome-terminal -- roslaunch apriltag_ros continuous_detection.launch
# gnome-terminal -- roslaunch husky_navigation gmapping.launch
# gnome-terminal -- rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# gnome-terminal -- rosbag record husky_pose
# gnome-terminal -- rosrun map_server map_saver -f /home/rahul/ros/arloc/src/arloc/description/maps/office

# For navigation

# gnome-terminal -- rosbag play husky_pose.bag
# gnome-terminal -- roslaunch husky_navigation amcl.launch
# gnome-terminal -- rosrun map_server map_server /home/rahul/ros/arloc/src/arloc/description/maps/office.yaml
# gnome-terminal -- rosbag record amcl_pose