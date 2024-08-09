#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <vector>
// #include <unordered_map>
#include <XmlRpcValue.h>
#include <iostream>

// Include the weightedAveragePose and bayesianFusion functions here

class PoseFusion {
public:
    PoseFusion() {
        marker_sub_ = nh_.subscribe("/tag_detections", 1, &PoseFusion::markerCallback, this);
        amcl_sub_ = nh_.subscribe("/amcl_pose", 1, &PoseFusion::amclCallback, this);
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

        // Set weights for weighted averaging
        weight_marker_ = 0.9; // High confidence in marker pose
        weight_amcl_ = 0.1; // Low confidence in AMCL pose

        // Set variances for Bayesian fusion (lower variance means higher confidence)
        // variance_marker_ = 0.01; // High confidence
        // variance_amcl_ = 0.1; // Low confidence
    }

    geometry_msgs::Pose weightedAveragePose(const geometry_msgs::Pose& pose_marker, const geometry_msgs::PoseWithCovarianceStamped& pose_amcl, double weight_marker, double weight_amcl) 
    {
        geometry_msgs::Pose fused_pose;

        // Weighted average for position
        fused_pose.position.x = (weight_marker * pose_marker.position.x + weight_amcl * pose_amcl.pose.pose.position.x) / (weight_marker + weight_amcl);
        fused_pose.position.y = (weight_marker * pose_marker.position.y + weight_amcl * pose_amcl.pose.pose.position.y) / (weight_marker + weight_amcl);
        fused_pose.position.z = (weight_marker * pose_marker.position.z + weight_amcl * pose_amcl.pose.pose.position.z) / (weight_marker + weight_amcl);

        // Weighted average for orientation (Quaternion) using SLERP
        tf::Quaternion quat_marker, quat_amcl, quat_fused;
        tf::quaternionMsgToTF(pose_marker.orientation, quat_marker);
        tf::quaternionMsgToTF(pose_amcl.pose.pose.orientation, quat_amcl);

        // Perform SLERP
        quat_fused = quat_marker.slerp(quat_amcl, weight_marker);

        // Convert back to geometry_msgs::Quaternion
        tf::quaternionTFToMsg(quat_fused, fused_pose.orientation);

        // WRONG METHOD: Weighted average for orientation (Quaternion)
        // fused_pose.orientation.x = (weight_marker * pose_marker.orientation.x + weight_amcl * pose_amcl.orientation.x) / (weight_marker + weight_amcl);
        // fused_pose.orientation.y = (weight_marker * pose_marker.orientation.y + weight_amcl * pose_amcl.orientation.y) / (weight_marker + weight_amcl);
        // fused_pose.orientation.z = (weight_marker * pose_marker.orientation.z + weight_amcl * pose_amcl.orientation.z) / (weight_marker + weight_amcl);
        // fused_pose.orientation.w = (weight_marker * pose_marker.orientation.w + weight_amcl * pose_amcl.orientation.w) / (weight_marker + weight_amcl);

        return fused_pose;

    }
    //camera to baselink : (in baselink frame) y:0.05 | x: -0.1 | z: -0.73523
    void backCalculate(const apriltag_ros::AprilTagDetectionArray& pose_marker)
    {
        XmlRpc::XmlRpcValue quaternion,position;
        std::vector<double> pos,quat;
        int marker_id = pose_marker.detections[0].id[0];

        nh_.getParam("/marker" + std::to_string(marker_id) + "/position",position);
        nh_.getParam("/marker" + std::to_string(marker_id) + "/orientation",quaternion);
        
        pos.push_back(static_cast<double>(position["x"]));
        pos.push_back(static_cast<double>(position["y"]));
        pos.push_back(static_cast<double>(position["z"]));
        
        quat.push_back(static_cast<double>(quaternion["x"]));
        quat.push_back(static_cast<double>(quaternion["y"]));
        quat.push_back(static_cast<double>(quaternion["z"]));
        quat.push_back(static_cast<double>(quaternion["w"]));
        
        tf2::Quaternion marker_world;
        marker_world.setW(quat[3]);
        marker_world.setX(quat[0]);
        marker_world.setY(quat[1]);
        marker_world.setZ(quat[2]);

        tf2::Quaternion cam_xyz;
        cam_xyz.setW(0);
        cam_xyz.setX(pose_marker.detections[0].pose.pose.pose.position.x);
        cam_xyz.setY(pose_marker.detections[0].pose.pose.pose.position.y);
        cam_xyz.setZ(pose_marker.detections[0].pose.pose.pose.position.z);
        
        tf2::Quaternion camera_marker;
        camera_marker.setW(pose_marker.detections[0].pose.pose.pose.orientation.w);
        camera_marker.setX(pose_marker.detections[0].pose.pose.pose.orientation.x);
        camera_marker.setY(pose_marker.detections[0].pose.pose.pose.orientation.y);
        camera_marker.setZ(pose_marker.detections[0].pose.pose.pose.orientation.z); 

        tf2::Quaternion camera_world = marker_world*camera_marker;  // means you get quaternion of camera in world_frame
        // double angle = ans.getAngle()*180/3.14159;

        // linear position transformation calculation
        tf2::Quaternion camera_world_xyz = camera_world * cam_xyz * camera_world.inverse();
        self_pose_.position.x = camera_world_xyz.getX() + pos[0] ;
        self_pose_.position.y = camera_world_xyz.getY() + pos[1] ;
        self_pose_.position.z = camera_world_xyz.getZ() + pos[2] ;
        self_pose_.orientation.x = camera_world.getX();
        self_pose_.orientation.y = camera_world.getY();
        self_pose_.orientation.z = camera_world.getZ();
        self_pose_.orientation.w = camera_world.getW();
        // tf2::quaternionTFToMsg(camera_world, self_pose_.orientation);
        // self_pose_.orientation = ;
    }

    void markerCallback(const apriltag_ros::AprilTagDetectionArray& msg) {
        pose_marker_ = msg;
        backCalculate(pose_marker_);
        // geometry_msgs::PoseWithCovariance marker_cov_pose = {pose_marker_.pose, variance_marker_};
        if (pose_amcl_received_) {
            // geometry_msgs::PoseWithCovariance amcl_cov_pose = {pose_amcl_.pose, variance_amcl_};

            // Weighted averaging fusion
            geometry_msgs::Pose fused_pose_avg = weightedAveragePose(self_pose_, pose_amcl_, weight_marker_, weight_amcl_);

            // Bayesian fusion
            // PoseWithCovariance fused_pose_bayes = bayesianFusion(marker_cov_pose, amcl_cov_pose);

            // Publish the fused pose to reinitialize AMCL
            geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
            initial_pose_msg.header = msg.header; // Use the same header
            initial_pose_msg.pose.pose = fused_pose_avg; // Using weighted average result here

            initial_pose_msg.pose.covariance[0] = weight_amcl_*weight_amcl_ * pose_amcl_.pose.covariance[0] + weight_marker_*weight_marker_ * 0.1; // Set the covariance for x
            initial_pose_msg.pose.covariance[7] = weight_amcl_*weight_amcl_ * pose_amcl_.pose.covariance[7] + weight_marker_*weight_marker_ * 0.1; // Set the covariance for y
            initial_pose_msg.pose.covariance[35] = weight_amcl_*weight_amcl_ * pose_amcl_.pose.covariance[35] + weight_marker_*weight_marker_ * 0.1; // Set the covariance for yaw

            // initial_pose_msg.pose.covariance[0] = fused_pose_bayes.variance; // Set the covariance for x
            // initial_pose_msg.pose.covariance[7] = fused_pose_bayes.variance; // Set the covariance for y
            // initial_pose_msg.pose.covariance[35] = fused_pose_bayes.variance; // Set the covariance for yaw
            pose_amcl_received_ = false;
            initial_pose_pub_.publish(initial_pose_msg);
        }
    }

    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        pose_amcl_ = *msg;
        pose_amcl_received_ = true;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber marker_sub_;
    ros::Subscriber amcl_sub_;
    ros::Publisher initial_pose_pub_;

    apriltag_ros::AprilTagDetectionArray pose_marker_;
    geometry_msgs::Pose self_pose_;
    geometry_msgs::PoseWithCovarianceStamped pose_amcl_;
    bool pose_amcl_received_ = false;

    double weight_marker_;
    double weight_amcl_;
    double variance_marker_;
    double variance_amcl_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_fusion");
    PoseFusion pose_fusion;
    // ros::spin();
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

//camera to baselink : (in baselink frame) y:0.05 | x: -0.1 | z: -0.73523

// To find orientation of robot
// first you know orientation of marker in world. then apply q.inverse where q is apriltag quaternion on marker abs orientation. Then you will camera orientation in world frame. which is
// (up X, right Y, forward Z) in world frame. Now orientation of base_link wrt to camera is fixed (left Y, forward X, up Z), So harcorde it to find robot orientation

// To find position of robot
// first you know relative axis of base_link and marker. So use pos in apriltag to get pos of marker in camera opencv frame and then represent it in baselink frame.
// having the oreintation of the robot in world frame and position of marker from base in baselink frame, you can find position of marker from base in world frame. then simply add all.