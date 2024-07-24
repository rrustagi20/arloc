# arloc
Robot Localisation improved using AR Tags (ARLoc)

A husky ground robot with enabled realsense depth camera is used for simulation for detection and pose estimation of april tags.

#### Cloning the repository
```bash
cd src
git clone git@github.com:rrustagi20/arloc.git .
git submodule init
git submodule update
cd .. && catkin build
source ../devel/setup.bash
```

## Resolving Warns / Issues
```bash
[ WARN] [1689018550.820283335]: Could not obtain transform from imu_link to base_footprint. Error was "imu_link" passed to lookupTransform argument source_frame does not exist. 

rosrun tf static_transform_publisher 1.0 1.0 1.0 0.0 0.0 1.0 imu_link base_footprint 1000
```
### Test Environment Preview
<!--![Screenshot from 2023-07-06 14-50-18](https://github.com/rrustagi20/slam/assets/77167720/f527c8e9-b948-4fcc-87b2-663b1619c495)-->
![image](https://github.com/user-attachments/assets/2a1f1cc0-bebc-410a-ba71-7ab4c5ec5e3b)

### Gmapping Map Preview
![image](https://github.com/user-attachments/assets/982a4023-3fc1-461a-b046-eae606c98a99)

### Dataset Preparation:
1) Mapping Phase
2) Localisation Phase

### Dependencies:
1) Husky (custom urdf file)
2) AprilTag

### Note:
1) Put real-time constraints on sensors for better explanation
