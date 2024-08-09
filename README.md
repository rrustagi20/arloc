# arloc
Robot Localisation improved using AR Tags (ARLoc)

A husky ground robot with enabled realsense depth camera is used for simulation for detection and pose estimation of april tags.

### Cloning the repository
```bash
cd src
git clone git@github.com:rrustagi20/arloc.git .
git submodule init
git submodule update
cd .. && catkin build
source ../devel/setup.bash
```
### Project Pipeline
![image](https://github.com/user-attachments/assets/339f63e0-c9a1-41e9-8f1d-916d35a71fed)

### Test Environment Preview
<!--![Screenshot from 2023-07-06 14-50-18](https://github.com/rrustagi20/slam/assets/77167720/f527c8e9-b948-4fcc-87b2-663b1619c495)-->
![image](https://github.com/user-attachments/assets/2a1f1cc0-bebc-410a-ba71-7ab4c5ec5e3b)

### Gmapping Map Preview
![image](https://github.com/user-attachments/assets/982a4023-3fc1-461a-b046-eae606c98a99)

### Dataset Preparation:
1) Mapping Phase
Run (1) Gmapping (2) ARTag detection library (3) Robot Movement Script (4) Record following topics
/tag_detections /map /map_metadata /tf /tf_static /front/scan /gazebo/model_states /imu/data /imu/data/bias
Save the map to publish it later during localisation

`rosbag record -o office_mapping_bag.bag /tag_detections /map /map_metadata /tf /tf_static /front/scan /gazebo/model_states /imu/data /imu/data/bias`

3) Localisation Phase
Publish the saved map using map_server package
Play the above bag file and run localisation algorithms (eg. amcl / als-ros)
Paralely record estimatedPose by the localisation algorithms to compare with the ground truth

`rosbag record -o amcl.bag /amcl_pose /particlecloud /tf /tf_static`


### Dependencies:
1) Husky (custom urdf file)
2) AprilTag
   
### Resolving Warns / Issues
```bash
[ WARN] [1689018550.820283335]: Could not obtain transform from imu_link to base_footprint. Error was "imu_link" passed to lookupTransform argument source_frame does not exist. 

rosrun tf static_transform_publisher 1.0 1.0 1.0 0.0 0.0 1.0 imu_link base_footprint 1000
```

### To Resolve:
1) Put real-time constraints on sensors for better explanation
2) Feels like main problem can pop up due to time calculation in pose_update code and due to varying freq of amcl_pose and husky_pose: update might be latency affected!
3) What is tf2::QuadWord?

### Learnings:
1. Configuring Transforms in World and Model frame.
```xml
<model name='Apriltag36_11_00005'>
<!-- <pose frame=''>0 0 0 0 0 0</pose> -->
<scale>1 1 1</scale>
    <link name='main'>

        <!-- (World Property) This one below changes (orients) both frames and marker together
    (Model Property Alone) The lower one changes only the frame. -->

        <!-- RPY is always along world axis in which models are placed -->

        <pose frame=''>3.05545 2.36463 0.5 0 1.5708 0</pose>
        <!-- <pose frame=''>3.15087 2.7088 0.5 0 0 1.57</pose> -->
        <velocity>0 0 0 0 -0 0</velocity>
        <acceleration>0 0 0 0 -0 0</acceleration>
        <wrench>0 0 0 0 -0 0</wrench>
    </link>
</model>
.
.
.
<model name='Apriltag36_11_00005'>
    <link name='main'>
    <pose frame=''>0 0 0 0 3.14159 1.5707</pose>
    <visual name='main_Visual'>
        <geometry>
        <box>
            <size>1 1 0.01</size>
        </box>
        </geometry>
        <material>
        <script>
            <uri>model://Apriltag36_11_00005/materials/scripts</uri>
            <uri>model://Apriltag36_11_00005/materials/textures</uri>
            <name>Apriltag36_11_00005</name>
        </script>
        </material>
    </visual>
    <self_collide>0</self_collide>
    <enable_wind>0</enable_wind>
    <kinematic>0</kinematic>
    </link>
    <static>1</static>
    <!-- <pose frame=''>3.15087 2.7088 0 0 -0 0</pose> -->
</model>
```
2. To publish desired /tf

a. Either publish the frame desired tf using static_tf_publisher or else

b. Check the view_frames.pdf for a tree of frames and blaming broadcaster node for it.

3. I was getting mirror pose (couldn't understand)

a. First I checked frame_id in /tag_detections to see if there is the issue. (check in rviz the tf and view_frames.pdf for parent publisher of frames)

b. Then I checked whether in pinhole model image is perceived as mirrored.

c. Then I checked rvec itself. Which had the error from there itself; meaning no influence of frame_id whatsoever. Cleared

d. Finally, I checked in solvePnP(objectPoints,ImagePoints,...) whether object and image points are considered in the same order which was not in my case and so after correcting that; the pose is now perfect as percieved by apriltag library.

4. The quaternion got in my apriltag_library is of marker wrt camera frame at that instant i.e by that much quaternion move the camera frame to obtain the marker frame; orientation from camera to marker

5. Use `rosbag play --clock abc.bag` with the `--clock` argument for forcing bag time to be published /clock and set `rosparam set /use_sim_time true` for nodes to use the simulated /clock time