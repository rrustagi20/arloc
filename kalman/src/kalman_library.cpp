#include <kalman/kalman_header.hpp>

namespace KF
{
    filtering::filtering(ros::NodeHandle &nh, std::string measured_pose, std::string corrected_pose, float RATE)
    {
        measuredPose = nh.subscribe(measured_pose,1,&filtering::poseCallBack,this);
        vel = nh.subscribe("/cmd_vel",1,&filtering::velCallBack,this);
        correctedPose = nh.advertise<apriltag_ros::AprilTagDetectionArray>(corrected_pose,1);
        dt=1.0/RATE;
        chk=0;
        firstRun=true;
        measurement = cv::Mat::zeros(2,1,CV_32F);
        ROS_INFO("filtering::filtering() successful");
    }
    filtering::~filtering()
    {
        ROS_INFO("filtering::~filtering() successful");
        return;
    }
    void filtering::poseCallBack(const apriltag_ros::AprilTagDetectionArray &pose)
    {
        ROS_INFO("filtering::poseCallBack() start");
        MarkerPose = pose;
        ROS_INFO("filtering::poseCallBack() success");
        chk++;
        run();
        return ;
    }
    void filtering::velCallBack(const geometry_msgs::Twist &vel)
    {
        ROS_INFO("filtering::velCallBack() start");
        MarkerVel = vel;
        chk++;
        ROS_INFO("filtering::velCallBack() success");
        return ;
    }
    void filtering::run()
    {
        ROS_INFO("filtering::run() start");
        if(firstRun)
        {
            setParams();
        }
        kf.statePre.at<float>(2)=MarkerVel.linear.x;
        kf.statePre.at<float>(3)=MarkerVel.linear.z;
        ROS_INFO("filtering::run() before prediction");
        prediction = kf.predict();
        ROS_INFO("filtering::run() after prediction & before updateMeasurement");
        updateMeasurement();
        ROS_INFO("filtering::run() after updateMeasurement & before correction");
        estimated = kf.correct(measurement);
        ROS_INFO("filtering::run() after correction & before publish");
        publish();
        ROS_INFO("filtering::run() success");
        return;
    }

    void filtering::setParams()
    {
        ROS_INFO("filtering::setParams() start");
        stateParams=4; //x,z,Vx,Vy  ~OmegaZ~
        measurementParams=2; //x,z
        controlParams=0;
        kf.init(stateParams,measurementParams,controlParams,CV_32FC1);
        ROS_INFO("filtering::setParams() 0");
        // measurement(measurementParams,1,CV_64F); //measurement
        // measurement = (cv::Mat_<float>(2,4) << 1,0,0,0, 0,1,0,0);
        kf.measurementMatrix = (cv::Mat_<float>(2,4) << 1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0);

        kf.transitionMatrix = (cv::Mat_<float>(4,4) << 1.0,0.0,0.0,-dt, 0.0,1.0,-dt,0.0,  0.0,0.0,1.0,0.0,  0.0,0.0,0.0,1.0);
        ROS_INFO("filtering::setParams() 1");
        // cv::setIdentity(kf.measurementMatrix);
        cv::setIdentity(kf.processNoiseCov,cv::Scalar::all(1e-4)); //adjust this for faster convergence - but higher noise
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1.0));
            
        ROS_INFO("filtering::setParams() 2");
        std::cout << MarkerPose.detections[0].pose.pose.pose.position.x << std::endl;
        kf.statePre.at<float>(0)=MarkerPose.detections[0].pose.pose.pose.position.x;
        kf.statePre.at<float>(1)=MarkerPose.detections[0].pose.pose.pose.position.z;
        kf.statePre.at<float>(2)=0.0;
        kf.statePre.at<float>(3)=0.0;
        // CV_8U: 0
        // CV_8S: 1
        // CV_16U: 2
        // CV_16S: 3
        // CV_32S: 4
        // CV_32F: 5
        // CV_64F: 6
        std::cout << kf.statePre.type() << std::endl;
        // kf.controlMatrix = (cv::Mat_<float>(4,1) << 1.0,0.0, 0.0,1.0);
        ROS_INFO("filtering::setParams() 3");
        // kf.statePre.at<float>(4)=0;
        firstRun=false;
        ROS_INFO("filtering::setParams() success");
        return;
    }
    void filtering::publish()
    {
        CorrectedPose = MarkerPose;
        CorrectedPose.detections[0].pose.pose.pose.position.x = 0.0;//estimated.at<float>(0);
        CorrectedPose.detections[0].pose.pose.pose.position.z = estimated.at<float>(1);
        correctedPose.publish(CorrectedPose);
        return;
    }
    void filtering::updateMeasurement()
    {
        measurement.at<float>(0) = MarkerPose.detections[0].pose.pose.pose.position.x;
        measurement.at<float>(1) = MarkerPose.detections[0].pose.pose.pose.position.z;
        return;
    }
}
