#include <pose_update/amcl_sampling.hpp>

namespace amcl
{

    AMCL_CLASS::AMCL_CLASS(ros::NodeHandle &nh)
    {
        ROS_INFO("AMCL_CLASS constructor called");
        marker_sub = nh.subscribe("/tag_detections", 1, &amcl::AMCL_CLASS::markerCallBack, this);
        // particlecloud_sub = nh.subscribe("/jetauto_1/particlecloud", 1, &amcl::AMCL_CLASS::particlecloudCallback, this);
        particlecloud_pub = nh.advertise<geometry_msgs::PoseArray>("/jetauto_1/particlecloud", 1);
        initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/jetauto_1/initialpose", 1);
        cx = pf_matrix_zero(); // A ZERO MATRIX 3x3
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                cx.m[i][j] = 0.0;
            }
        }
        cx.m[0][0] = 0.1;
        cx.m[1][1] = 0.1;
        cx.m[2][2] = 0.1;
        chk=false;
        pdf = pf_pdf_gaussian_alloc(mean, cx);
        ROS_INFO("AMCL_CLASS constructor finished");
    }
    
    AMCL_CLASS::~AMCL_CLASS()
    {
        // Destroy the pdf
        // pf_pdf_gaussian_free(pdf);
    }

    void AMCL_CLASS::markerCallBack(const apriltag_ros::AprilTagDetectionArray &pose)
    {
        ROS_INFO("Marker callback called");
        if(pose.detections.empty())
            return ;
        else
        {
            mean = pose_from_id(pose.detections[0].id[0]);
        }
        ROS_INFO("Marker callback finished");
        return ;
    }

    pf_vector_t AMCL_CLASS::pose_from_id(int id)
    {
        // Read the file and get the mean
        ROS_INFO("Pose from id called");
        std::ifstream inputFile("marker_pose/src/marker_pose.txt");
        bool ans;
        if(!inputFile.is_open())
        {
            std::cout << "Failed to open the file." << std::endl;
            exit(1); 
        }
        else
        {
            std::string line;
            int chk_id; 
            float x, y, z;
            char ch;
            while (std::getline(inputFile, line)) 
            {
                inputFile >> chk_id >> ch >> x >> ch >> y >> ch >> z;
                if(chk_id == id)
                {
                    mean.v[0] = x;
                    mean.v[1] = y;
                    mean.v[2] = z;
                    break;
                }
                else
                    continue;
            }
            inputFile.close();
        }
        // Allot a gaussian pdf
        // pdf = pf_pdf_gaussian_alloc(mean, cx);
        // chk=true;
        publish_pose();
        std::cout <<chk<<std::endl;
        ROS_INFO("Pose from id finished");
        return mean;
    }
    void AMCL_CLASS::publish_pose()
    {
        ROS_INFO("Publish pose called");
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "/jetauto_1/map";
        pose_msg.pose.pose.position.x = mean.v[0];
        pose_msg.pose.pose.position.y = mean.v[1];
        pose_msg.pose.pose.position.z = mean.v[2];
        // pose_msg.pose.pose.orientation.x = 0.0;
        // pose_msg.pose.pose.orientation.y = 0.0;
        // pose_msg.pose.pose.orientation.z = 0.0;
        // pose_msg.pose.pose.orientation.w = 1.0;
        pose_msg.pose.covariance[0] = 0.1;
        pose_msg.pose.covariance[7] = 0.1;
        pose_msg.pose.covariance[35] = 0.1;
        initialpose_pub.publish(pose_msg);
        ROS_INFO("Publish pose finished");
        return ;
    }
    // void AMCL_CLASS::particlecloudCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
    // {
        // std::cout <<chk<<std::endl;
        //     ROS_INFO("Particlecloud callback called");
        // if(chk)
        // {
        //     ROS_INFO("Particlecloud callback called");
        //     // ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
        //     poseArray = *msg;
        //     // ROS_INFO("Total particlecloud captured: [%d]", poseArray.poses.size());
        //     for(auto i : poseArray.poses)
        //     {
        //         pf_vector_t sample = sampler();
        //         // ROS_INFO("x: [%f], y: [%f], z: [%f]", i.position.x, i.position.y, i.position.z);
        //         i.position.x = sample.v[0];
        //         i.position.z = sample.v[1];
        //         i.orientation.z = sample.v[2];
        //     }
        //     particlecloud_pub.publish(poseArray);
        //     ROS_INFO("Particlecloud callback finished");
        //     chk=false;
        // }
        // return;
    // }

    // pf_vector_t AMCL_CLASS::sampler()
    // {
        // // Generate a sample from the pdf.
        // ROS_INFO("Sampler called");
        // pf_vector_t sample = pf_pdf_gaussian_sample(pdf);
        // ROS_INFO("Sampler finished");
        // return sample;
    // }
}