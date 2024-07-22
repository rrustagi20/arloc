#include <kalman/header.hpp>

int dynamParams = 5;
int measureParams = 5;
int controlParams = 5; // considering v_x, v_y, omega_z
float dt = 0.033;

cv::KalmanFilter kf(dynamParams, measureParams, controlParams, CV_64F);


void markerposeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 5; i++)
        kf.temp1.at<double>(i,0) = msg->data[i];
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // kf.temp1[0] = msg->pose.position.x
    std::vector<double> pose,vel;
    pose.push_back(msg->pose.pose.position.x); // x
    pose.push_back(msg->pose.pose.position.y); // y
    vel.push_back(msg->twist.twist.linear.x); // V_x
    vel.push_back(msg->twist.twist.linear.y); // V_y
    vel.push_back(msg->twist.twist.angular.z); // omega_z

    
    kf.predict();

    pose.clear();
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"kalman_lib");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe("/pose", 1, poseCallback);
    ros::Subscriber markerpose_sub = nh.subscribe("/markerpose", 1, markerposeCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/corrected_markerpose", 1);


    kf.controlMatrix = cv::Mat::eye(dynamParams, controlParams, CV_64F);
    for(int i=0; i < 2 ; i++)
        kf.controlMatrix.at<double>(i,i) = dt;

    kf.transitionMatrix = cv::Mat::eye(dynamParams, dynamParams, CV_64F);
    kf.measurementMatrix = cv::Mat::eye(measureParams, dynamParams, CV_64F);

    kf.temp1 = cv::Mat::eye(dynamParams, 1, CV_64F);

    kf.measurementNoiseCov = cv::Mat::eye(measureParams, measureParams, CV_64F);
    kf.processNoiseCov = cv::Mat::eye(dynamParams, dynamParams, CV_64F);

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}