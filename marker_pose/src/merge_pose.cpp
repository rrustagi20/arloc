#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

// Global variables to store the latest AMCL and independent pose estimates
geometry_msgs::PoseWithCovarianceStamped amcl_pose;
geometry_msgs::PoseWithCovarianceStamped independent_pose;
ros::Publisher pub;
void fusePoses();

// Callback function to receive AMCL pose estimates
void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    amcl_pose = *msg;
}

// Callback function to receive independent pose estimates
void independentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    independent_pose = *msg;
    
    // Fuse the independent pose with AMCL pose estimate
    fusePoses();
}

// Function to fuse the independent pose with AMCL pose estimate
void fusePoses() {
    // Extract the mean and covariance from AMCL and independent pose estimates
    tf::Pose amcl_mean, independent_mean;
    float amcl_cov[36], independent_cov[36];
    tf::poseMsgToTF(amcl_pose.pose.pose, amcl_mean);
    tf::poseMsgToTF(independent_pose.pose.pose, independent_mean);
    for (int i = 0; i < 36; i++) {
        amcl_cov[i] = amcl_pose.pose.covariance[i];
        independent_cov[i] = independent_pose.pose.covariance[i];
    }

    // Calculate the fused mean
    tf::Pose fused_mean;
    double amcl_weight = 0.1; // Weight for AMCL pose (lower means more trust in independent pose)
    double independent_weight = 0.9; // Weight for independent pose

    // Weighted average of translations
    fused_mean.getOrigin() = amcl_mean.getOrigin() * amcl_weight + independent_mean.getOrigin() * independent_weight;

    // Weighted average of rotations (using quaternion slerp)
    tf::Quaternion amcl_quat = amcl_mean.getRotation();
    tf::Quaternion independent_quat = independent_mean.getRotation();
    fused_mean.setRotation(amcl_quat.slerp(independent_quat, independent_weight));

    // Calculate the fused covariance
    float fused_cov[36];
    for (int i = 0; i < 36; i++) {
        fused_cov[i] = amcl_cov[i] * amcl_weight + independent_cov[i] * independent_weight;
    }
    geometry_msgs::PoseWithCovarianceStamped fused_pose;
    tf::poseTFToMsg(fused_mean, fused_pose.pose.pose);
    for (int i = 0; i < 36; i++) {
        fused_pose.pose.covariance[i] = fused_cov[i];
    }
    // Publish the fused pose estimate
    pub.publish(fused_pose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_fusion_node");
    ros::NodeHandle nh;

    // Subscribe to AMCL and independent pose topics
    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1, amclPoseCallback);
    ros::Subscriber independent_sub = nh.subscribe("/independent_pose", 1, independentPoseCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initial_pose", 1);

    ros::spin();
    return 0;
}


void AMCLOdom::initializePoseFromTF(double sx, double sy)
{
  // If the fused pose is available, use it to initialize the particle set
  if (fused_pose_.pose.pose.position.x != 0.0 || fused_pose_.pose.pose.position.y != 0.0)
  {
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = fused_pose_.pose.pose.position.x;
    pf_init_pose_mean.v[1] = fused_pose_.pose.pose.position.y;
    pf_init_pose_mean.v[2] = angles::normalize_angle(tf2::getYaw(fused_pose_.pose.pose.orientation));

    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init_pose_cov.m[0][0] = fused_pose_.pose.covariance[0];
    pf_init_pose_cov.m[1][1] = fused_pose_.pose.covariance[7];
    pf_init_pose_cov.m[2][2] = fused_pose_.pose.covariance[35];

    pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
    pf_init_ = false;
  }
}