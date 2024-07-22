#include <kalman/header.hpp>

namespace KF
{
    class filtering
    {
        public:
            filtering(ros::NodeHandle &, std::string, std::string, float);
            ~filtering();
            void poseCallBack(const apriltag_ros::AprilTagDetectionArray &);
            void velCallBack(const geometry_msgs::Twist &);
            void updateMeasurement();
            void setParams();
            void publish();
            void run();

            // kf.predict()            :: predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
            // kf.correct(measurement) :: corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
            // z(k) :: measurement (observation) at k
            // H(k) :: measurement matrix

            apriltag_ros::AprilTagDetectionArray MarkerPose;
            apriltag_ros::AprilTagDetectionArray CorrectedPose;
            geometry_msgs::Twist MarkerVel;

            ros::Subscriber measuredPose;
            ros::Subscriber vel;
            ros::Publisher correctedPose;

            cv::KalmanFilter kf;
            cv::Mat prediction,estimated,measurement;

            int stateParams; //x,z,Vx,Vy,OmegaZ
            int measurementParams; //x,z
            int controlParams;

            float dt;
            int chk;
            bool firstRun;
    };
}