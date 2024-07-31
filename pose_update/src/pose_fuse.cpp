#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
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

    // geometry_msgs::Pose weightedAveragePose(const geometry_msgs::Pose& pose_marker, const geometry_msgs::PoseWithCovarianceStamped& pose_amcl, double weight_marker, double weight_amcl) 
    // {
    //     geometry_msgs::Pose fused_pose;

    //     // Weighted average for position
    //     fused_pose.position.x = (weight_marker * pose_marker.position.x + weight_amcl * pose_amcl.pose.pose.position.x) / (weight_marker + weight_amcl);
    //     fused_pose.position.y = (weight_marker * pose_marker.position.y + weight_amcl * pose_amcl.pose.pose.position.y) / (weight_marker + weight_amcl);
    //     fused_pose.position.z = (weight_marker * pose_marker.position.z + weight_amcl * pose_amcl.pose.pose.position.z) / (weight_marker + weight_amcl);

    //     // Weighted average for orientation (Quaternion) using SLERP
    //     tf::Quaternion quat_marker, quat_amcl, quat_fused;
    //     tf::quaternionMsgToTF(pose_marker.orientation, quat_marker);
    //     tf::quaternionMsgToTF(pose_amcl.pose.pose.orientation, quat_amcl);

    //     // Perform SLERP
    //     quat_fused = quat_marker.slerp(quat_amcl, weight_marker);

    //     // Convert back to geometry_msgs::Quaternion
    //     tf::quaternionTFToMsg(quat_fused, fused_pose.orientation);

    //     // WRONG METHOD: Weighted average for orientation (Quaternion)
    //     // fused_pose.orientation.x = (weight_marker * pose_marker.orientation.x + weight_amcl * pose_amcl.orientation.x) / (weight_marker + weight_amcl);
    //     // fused_pose.orientation.y = (weight_marker * pose_marker.orientation.y + weight_amcl * pose_amcl.orientation.y) / (weight_marker + weight_amcl);
    //     // fused_pose.orientation.z = (weight_marker * pose_marker.orientation.z + weight_amcl * pose_amcl.orientation.z) / (weight_marker + weight_amcl);
    //     // fused_pose.orientation.w = (weight_marker * pose_marker.orientation.w + weight_amcl * pose_amcl.orientation.w) / (weight_marker + weight_amcl);

    //     return fused_pose;

    // }

    //camera to baselink : (in baselink frame) y:0.05 | x: -0.1 | z: -0.73523
    void backCalculate(const tf2::Quaternion& pos, const tf2::Quaternion& quat, const tf2::Quaternion& ori_marker_wrt_camera, const tf2::Quaternion& pos_marker_wrt_image)
    {        
        // Calulate Robot Orientation in World Frame
        tf2::Quaternion ori_marker_wrt_world = quat;
        // camera_marker.setW(pose_marker.detections[0].pose.pose.pose.orientation.w);
        // camera_marker.setX(pose_marker.detections[0].pose.pose.pose.orientation.x);
        // camera_marker.setY(pose_marker.detections[0].pose.pose.pose.orientation.y);
        // camera_marker.setZ(pose_marker.detections[0].pose.pose.pose.orientation.z); 

        tf2::Quaternion camera_world = ori_marker_wrt_world*(ori_marker_wrt_camera.inverse());  // means you get quaternion of camera in world_frame
        // double angle = ans.getAngle()*180/3.14159;
        // Now hardcoding relative rotation bw robot and base
        tf2::Quaternion ori_cam_wrt_base = tf2::Quaternion(0,0,-0.7071,0.7071)*tf2::Quaternion(-0.7071,0,0,0.7071);
        tf2::Quaternion ori_robot_wrt_world = camera_world*(ori_cam_wrt_base.inverse());
        // ori_robot_wrt_world = ori_robot_wrt_world.normalize();
        // Calculate Robot Position in World Frame
        tf2::Quaternion pos_cam_wrt_base(0.1,-0.05,0.73523,0);
        tf2::Quaternion pos_marker_wrt_base_fromcam(pos_marker_wrt_image[2],-pos_marker_wrt_image[0],-pos_marker_wrt_image[1],0);
        tf2::Quaternion pos_marker_wrt_base_frombase(pos_marker_wrt_base_fromcam[0]+pos_cam_wrt_base[0],pos_marker_wrt_base_fromcam[1]+pos_cam_wrt_base[1],pos_marker_wrt_base_fromcam[2]+pos_cam_wrt_base[2],0);
        tf2::Quaternion pos_diff_wrt_world = ori_robot_wrt_world*pos_marker_wrt_base_frombase*(ori_robot_wrt_world.inverse());
        // tf2::Vector3 pso = tf2::quatRotate(ori_robot_wrt_world.inverse(),tf2::Vector3(pos_marker_wrt_base_frombase[0],pos_marker_wrt_base_frombase[1],pos_marker_wrt_base_frombase[2]));
        // cam_xyz.setW(0);
        // cam_xyz.setX(pose_marker.detections[0].pose.pose.pose.position.x);
        // cam_xyz.setY(pose_marker.detections[0].pose.pose.pose.position.y);
        // cam_xyz.setZ(pose_marker.detections[0].pose.pose.pose.position.z);

        // linear position transformation calculation
        // tf2::Quaternion camera_world_xyz = camera_world * cam_xyz * camera_world.inverse();

        self_pose_.position.x = -pos_diff_wrt_world.getX() + pos.getX() ;
        self_pose_.position.y = -pos_diff_wrt_world.getY() + pos.getY() ;
        self_pose_.position.z = 0.1322 ;
        self_pose_.orientation.x = ori_robot_wrt_world.getX();
        self_pose_.orientation.y = ori_robot_wrt_world.getY();
        self_pose_.orientation.z = ori_robot_wrt_world.getZ();
        self_pose_.orientation.w = ori_robot_wrt_world.getW();
        // ROS_INFO("Position: %f %f %f",pos_marker_wrt_base_frombase[0],pos_marker_wrt_base_frombase[1],pos_marker_wrt_base_frombase[2]);
        // std::cout << "Position: " << pso[0] << " " << pso[1] << std::endl;
        // ROS_INFO("Position: %f %f %f",self_pose_.position.x,self_pose_.position.y,self_pose_.position.z);
        // ROS_INFO("--------");
        // ROS_INFO("Orientation: %f %f %f %f",ori_marker_wrt_world.getX(),ori_marker_wrt_world.getY(),ori_marker_wrt_world.getZ(),ori_marker_wrt_world.getW());

        // std::cout << "Position: " << pos.getX() << " " << pos.getY() << " " << self_pose_.position.z << std::endl;
        // tf2::quaternionTFToMsg(camera_world, self_pose_.orientation);
        // self_pose_.orientation = ;
    }

    // geometry_msgs::PoseWithCovarianceStamped mapFramePose()
    // {
    //     // geometry_msgs::PoseWithCovarianceStamped pose;
    //     // pose.pose.pose = self_pose_;
    //     // pose.header.frame_id = "odom";
    //     // pose.header.stamp = ros::Time::now();
    //         // float waitTime = 0.052;
    //     float waitTime = 0.1;
    //     tf2_ros::Buffer tfBuffer;
    //     tf2_ros::TransformListener tfListener(tfBuffer);
    //     geometry_msgs::PoseWithCovarianceStamped transformed_pose;
    //     // geometry_msgs::TransformStamped transformStamped;
    //     try
    //     {
    //         // Wait for the transformation between the aruco marker frame and the world frame
    //         // tfBuffer.allFramesAsString();
    //         ros::Duration(waitTime).sleep();
    //         geometry_msgs::TransformStamped mapToOdom = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
    //         mapToOdom.transform.rotation.
    //         tf2::doTransform(self_pose_, transformed_pose, mapToOdom);
    //         transformed_pose.header.frame_id = "map";
    //         transformed_pose.header.stamp = ros::Time::now();
    //     }
    //     catch (tf2::TransformException &ex)
    //     {
    //         ROS_WARN("%s", ex.what());
    //     }
    //     return transformed_pose;
    // }

    void markerCallback(const apriltag_ros::AprilTagDetectionArray& msg) 
    {
        if(msg.detections.empty())
            return;

        XmlRpc::XmlRpcValue quaternion,position;
        tf2::Quaternion pos,quat;

        int marker_id = msg.detections[0].id[0];

        nh_.getParam("/marker" + std::to_string(marker_id) + "/position",position);
        nh_.getParam("/marker" + std::to_string(marker_id) + "/orientation",quaternion);
        
        pos.setX(static_cast<double>(position["x"]));
        pos.setY(static_cast<double>(position["y"]));
        // pos.setZ(static_cast<double>(position["z"]));
        
        quat.setX(static_cast<double>(quaternion["x"]));
        quat.setY(static_cast<double>(quaternion["y"]));
        quat.setZ(static_cast<double>(quaternion["z"]));
        quat.setW(static_cast<double>(quaternion["w"]));
        tf2::Quaternion ori_marker_wrt_camera(msg.detections[0].pose.pose.pose.orientation.x,msg.detections[0].pose.pose.pose.orientation.y,msg.detections[0].pose.pose.pose.orientation.z,msg.detections[0].pose.pose.pose.orientation.w);
        tf2::Quaternion pos_marker_wrt_image(msg.detections[0].pose.pose.pose.position.x,msg.detections[0].pose.pose.pose.position.y,msg.detections[0].pose.pose.pose.position.z,0);
        // pose_marker_ = msg;
        backCalculate(pos,quat,ori_marker_wrt_camera,pos_marker_wrt_image);
        // geometry_msgs::PoseWithCovariance marker_cov_pose = {pose_marker_.pose, variance_marker_};
        // if (pose_amcl_received_) {
            // geometry_msgs::PoseWithCovariance amcl_cov_pose = {pose_amcl_.pose, variance_amcl_};

            // Weighted averaging fusion
            // geometry_msgs::Pose fused_pose_avg = weightedAveragePose(self_pose_, pose_amcl_, weight_marker_, weight_amcl_);

            // Bayesian fusion
            // PoseWithCovariance fused_pose_bayes = bayesianFusion(marker_cov_pose, amcl_cov_pose);

            // Publish the fused pose to reinitialize AMCL
            // geometry_msgs::PoseWithCovarianceStamped initial_pose_msg = mapFramePose();
            geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
            // initial_pose_msg.header = msg.header; // Use the same header
            initial_pose_msg.header.frame_id = "map";
            initial_pose_msg.header.stamp = ros::Time::now();
            initial_pose_msg.pose.pose = self_pose_; // Using weighted average result here

            // initial_pose_msg.pose.covariance[0] = weight_amcl_*weight_amcl_ * pose_amcl_.pose.covariance[0] + weight_marker_*weight_marker_ * 0.01; // Set the covariance for x
            // initial_pose_msg.pose.covariance[7] = weight_amcl_*weight_amcl_ * pose_amcl_.pose.covariance[7] + weight_marker_*weight_marker_ * 0.01; // Set the covariance for y
            // initial_pose_msg.pose.covariance[35] = weight_amcl_*weight_amcl_ * pose_amcl_.pose.covariance[35] + weight_marker_*weight_marker_ * 0.01; // Set the covariance for yaw

            initial_pose_msg.pose.covariance[0] = 0.01;
            initial_pose_msg.pose.covariance[7] = 0.01;
            initial_pose_msg.pose.covariance[35] = 0.01;

            // initial_pose_msg.pose.covariance[0] = fused_pose_bayes.variance; // Set the covariance for x
            // initial_pose_msg.pose.covariance[7] = fused_pose_bayes.variance; // Set the covariance for y
            // initial_pose_msg.pose.covariance[35] = fused_pose_bayes.variance; // Set the covariance for yaw
            pose_amcl_received_ = false;
            initial_pose_pub_.publish(initial_pose_msg);
        // }
    }

    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        pose_amcl_ = *msg;
        if(msg->pose.pose.position.x == 0 && msg->pose.pose.position.y == 0)
            return;
        pose_amcl_received_ = true;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber marker_sub_;
    ros::Subscriber amcl_sub_;
    ros::Publisher initial_pose_pub_;

    // apriltag_ros::AprilTagDetectionArray pose_marker_;
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
// (forw X, right Y, down Z) in world frame. Now orientation of base_link wrt to camera is fixed (left Y, forward X, up Z), So harcorde it to find robot orientation

// To find position of robot
// first you know relative axis of base_link and marker. So use pos in apriltag to get pos of marker in camera opencv frame and then represent it in baselink frame.
// having the oreintation of the robot in world frame and position of marker from base in baselink frame, you can find position of marker from base in world frame. then simply add all.

