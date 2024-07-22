#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <vector>
#include <eigen3/Eigen/Dense>

Eigen::Matrix3d R_cam_base; // Rotation from marker to base_link
tf2::Quaternion T_abs; // Absolute Translation from marker to cam in world frame
tf2::Quaternion T_marker_cam; // Translation from marker to camera
tf2::Quaternion T_abs_; // Absolute Translation from cam to base to world frame
tf2::Quaternion R_marker_cam; // Rotation from marker to camera
tf2::Quaternion R_base_world; // Rotation from base_link to world
tf2::Quaternion T_base_world; // Translation from base_link to world
tf2::Quaternion T_map_odom; // Translation from odom to map frame
tf2::Quaternion R_odom_base; // Rotation from odom to map frame
std::vector<geometry_msgs::TransformStamped> transformStamped; // [T_marker_base,T_base_world]
// ros::init(argc, argv, "aruco_pose_transformer");
// ros::NodeHandle nh;

void metaData()
{
    // Define the transformation between the image/marker frame and the base_link frame
    R_cam_base << 0, 0, 1,
                 -1, 0, 0,
                  0,-1, 0;
    T_marker_cam.setValue(0, 0, 0, 0);
    T_base_world.setValue(0, 0, 0, 0);
    // Use the transformation to transform the Aruco pose to the world coordinate system
}

void estimateRobotPose()
{
    // Estimate the pose of the Robot in World Frame using Gmapping information
    // Your code here...

    T_base_world.setX(transformStamped[1].transform.translation.x);
    T_base_world.setY(transformStamped[1].transform.translation.y);
    T_base_world.setZ(transformStamped[1].transform.translation.z);

    T_map_odom.setX(transformStamped[2].transform.translation.x);
    T_map_odom.setY(transformStamped[2].transform.translation.y);
    T_map_odom.setZ(transformStamped[2].transform.translation.z);

    R_odom_base.setX(transformStamped[2].transform.rotation.x);
    R_odom_base.setY(transformStamped[2].transform.rotation.y);
    R_odom_base.setZ(transformStamped[2].transform.rotation.z);
    R_odom_base.setW(transformStamped[2].transform.rotation.w);

    T_base_world = T_base_world + (R_odom_base.inverse()*T_map_odom*R_odom_base);    
}

void initVals()
{
    // transformStamped[0].getOrigin();
    R_marker_cam.setX(transformStamped[0].transform.rotation.x);
    R_marker_cam.setY(transformStamped[0].transform.rotation.y);
    R_marker_cam.setZ(transformStamped[0].transform.rotation.z);
    R_marker_cam.setW(transformStamped[0].transform.rotation.w);

    R_base_world.setX(transformStamped[1].transform.rotation.x);
    R_base_world.setY(transformStamped[1].transform.rotation.y);
    R_base_world.setZ(transformStamped[1].transform.rotation.z);
    R_base_world.setW(transformStamped[1].transform.rotation.w);

    T_marker_cam.setX(transformStamped[0].transform.translation.x);
    T_marker_cam.setY(transformStamped[0].transform.translation.y);
    T_marker_cam.setZ(transformStamped[0].transform.translation.z);
}

void findTransforms(std::vector<geometry_msgs::TransformStamped>& transformStamped, int id)
{
    float waitTime = 0.052;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(waitTime).sleep();
    // geometry_msgs::TransformStamped transformStamped;
    try
    {
        // Wait for the transformation between the aruco marker frame and the world frame
        // tfBuffer.allFramesAsString();
        // tfBuffer.waitForTransform("/base_footprint", "/odom", ros::Time::now(), ros::Duration(4.0));
        // Only detecting Tag 3
        // transformStamped.push_back(tfBuffer.lookupTransform("t3", "camera_realsense_gazebo", ros::Time(0)));
        // Write code to detect all possible
        // for(auto i : {"t1","t2","t3","t4","t5","t6"})
        transformStamped.push_back(tfBuffer.lookupTransform(std::to_string(id), "camera_realsense_gazebo", ros::Time(0)));
        transformStamped.push_back(tfBuffer.lookupTransform("base_link", "odom", ros::Time(0)));
        transformStamped.push_back(tfBuffer.lookupTransform("map", "odom", ros::Time(0)));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

void update(int ID)
{
    // Update the transformation between the base_link frame and the world frame
    ROS_INFO("Inside update function");
    while(transformStamped.size() != 3)
    {
        transformStamped.clear();
        findTransforms(transformStamped,ID);
    }

    // Initialize the values of the transformation between the image/marker frame and the base_link frame
    initVals();
    estimateRobotPose();
    ROS_INFO("After estimateRobotPose");
    T_abs = (R_marker_cam.inverse()*T_marker_cam*R_marker_cam);
    // T_abs = -T_abs; // Offset Management
    // Conversion form camera to absframe
    T_abs_.setX(-T_abs.getZ());
    T_abs_.setY(T_abs.getX());
    T_abs_.setZ(T_abs.getY());

    T_abs_ = (R_base_world.inverse()*T_abs_*R_base_world);
    // T_abs = T_marker_cam;
    T_abs_ = T_abs_ + T_base_world;
    
    ROS_INFO("x: %f | y: %f | z: %f",T_abs_.getX(),T_abs_.getY(),T_abs_.getZ());

    std::string param_name = "/marker_" + std::to_string(ID) + "/position";

    if (!ros::param::has(param_name)) {

        std::vector<double> position = {T_abs_.getX(),T_abs_.getY(),T_abs_.getZ()};
        std::vector<double> orientation = {0.0, 0.0, 0.0, 1.0};
        ros::param::set("/marker_" + std::to_string(ID) + "/position", position);
        ros::param::set("/marker_" + std::to_string(ID) + "/orientation", orientation);
    }
}

void markerCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    int markerID_detected = 0;
    apriltag_ros::AprilTagDetectionArray markerPose = *msg;
    markerID_detected = markerPose.detections[0].id[0];
    update(markerID_detected);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_pose_transform");
    ros::NodeHandle nh;
    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 1, markerCallback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        // Use transformStamped to transform Aruco pose to the world coordinate system
        // Your Aruco pose transformation code here...
        // update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}