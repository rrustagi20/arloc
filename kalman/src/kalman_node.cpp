#include <kalman/kalman_header.hpp>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"kalman");

    ros::NodeHandle nh;

    std::string measured_pose = "/tag_detections";
    std::string corrected_pose = "/corrected/pose";
    
    float RATE=30;

    KF::filtering filter(nh,measured_pose,corrected_pose,RATE);

    ros::Rate rate(RATE);
    ROS_INFO("starting now: entering ros::ok()");
    while(ros::ok())
    {
        ros::spinOnce();

        // filter.prediction = filter.kf.predict();

        // predictedPt(prediction[0],prediction[1]);

        // filter.updateMeasurement();

        // measurement.at<float>(0) = 0;//MarkerPose.pose.x;
        // measurement.at<float>(1) = 0;//MarkerPose.pose.y;

        // filter.estimated = filter.kf.correct(filter.measurement);
        // estimatedPt(estimated(0),estimated(1));
        // filter.publish();
        // filter.run();
        rate.sleep();
    }
    filter.~filtering();
    return 0;
}