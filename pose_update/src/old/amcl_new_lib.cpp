#include <pose_update/headers.hpp>

geometry_msgs::PoseArray poseArray;
bool run_particle=false;
std::string names[] = {"t0", "t1", "t2", "t3", "t4", "t5", "t6"};


void particlecloudCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
    poseArray = *msg;
    ROS_INFO("Total particlecloud captured: [%d]", poseArray.poses.size());
    for(auto i : poseArray.poses)
    {
        // ROS_INFO("x: [%f], y: [%f], z: [%f]", i.position.x, i.position.y, i.position.z);
        
    }
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amcl_new_lib");
    ros::NodeHandle nh;
    ROS_INFO("Loading amcl_new_lib...");
    ros::Subscriber amcl_sub = nh.subscribe("/particlecloud", 1, &particlecloudCallback);
    ros::Subscriber marker_sub = nh.subscribe("/tag_detections", 1, &particlecloudCallback);
    // ros::Subscriber sub2 = nh.subscribe("/tf", 1, &transformCallback);
    return 0;
}