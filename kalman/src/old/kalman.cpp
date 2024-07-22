#include <kalman/header.hpp>

void poseCallBack(const detection::pose_struct pose)
{
    detection::pose_struct MarkerPose;
    MarkerPose.pose.x=pose.pose.x;
    MarkerPose.pose.y=pose.pose.y;
    MarkerPose.pose.z=pose.pose.z;
    return ;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"kalman");
    ros::NodeHandle nh;
    ros::Subscriber measuredPose = nh.subscribe("/detected/pose",1,&poseCallBack);
    ros::Publisher correctedPose = nh.advertise<detection::pose_struct>("/corrected/pose",1);
    
    cv::KalmanFilter filter;
    cv::Mat prediction,estimated;
    // cv::Point2f* predictedPt, estimatedPt;

    int stateParams=5; //x,z,Vx,Vy,OmegaZ
    int measurementParams=2; //x,z
    int controlParams=0;

    filter.init(stateParams,measurementParams,controlParams,CV_64F);

    // cv::Mat state(stateParams,1,CV_64F);
    cv::Mat measurement(measurementParams,1,CV_64F);

    // filter.transitionMatrix = cv::Mat::eye(stateParams,stateParams,CV_64F);
    float dt=1.0/30.0;
    filter.transitionMatrix = (cv::Mat_<float>(4,4) << 1,0,0,0, dt,1,0,0,  0,0,1,0,  0,0,dt,1);
    cv::setIdentity(filter.measurementMatrix);
    cv::setIdentity(filter.processNoiseCov,cv::Scalar::all(1e-4)); //adjust this for faster convergence - but higher noise
    cv::setIdentity(filter.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(filter.errorCovPost, cv::Scalar::all(1));

    // for(int i=0;i<stateParams;i++)
    filter.statePre.at<float>(0)=0;
    filter.statePre.at<float>(1)=0;
    filter.statePre.at<float>(2)=0;
    filter.statePre.at<float>(3)=0;
    filter.statePre.at<float>(4)=0;

    ros::Rate rate(30);
    
    while(ros::ok())
    {
        ros::spinOnce();

        prediction = filter.predict();
        // predictedPt(prediction[0],prediction[1]);

        measurement.at<float>(0) = 0;//MarkerPose.pose.x;
        measurement.at<float>(1) = 0;//MarkerPose.pose.y;

        estimated = filter.correct(measurement);
        // estimatedPt(estimated(0),estimated(1));

        rate.sleep();
    }
    return 0;
}
