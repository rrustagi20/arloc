#include <pose_update/headers.hpp>
#include <amcl/pf/pf.h>
#include <amcl/pf/pf_pdf.h>
#include <amcl/AMCLConfig.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

namespace amcl
{
    class AMCL_CLASS
    {
        public:
            AMCL_CLASS(ros::NodeHandle &);
            ~AMCL_CLASS();
            void markerCallBack(const apriltag_ros::AprilTagDetectionArray &);
            void particlecloudCallback(const geometry_msgs::PoseArray::ConstPtr& );
            pf_vector_t pose_from_id(int );
            void publish_pose();
            // pf_vector_t sampler();
        
            ros::Subscriber marker_sub;
            ros::Subscriber particlecloud_sub;
            ros::Publisher particlecloud_pub;
            ros::Publisher initialpose_pub;

            // geometry_msgs::PoseArray poseArray;
            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pf_vector_t mean;
            pf_matrix_t cx;
            pf_pdf_gaussian_t *pdf;
            bool chk;
    };
}