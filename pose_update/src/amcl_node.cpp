#include <pose_update/amcl_sampling.hpp>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "pose_update");
    ros::NodeHandle nh;

    ros::Rate loop_rate(30);

    amcl::AMCL_CLASS amcl(nh);
    ROS_INFO("AMCL CLASS INITIALIZED...");
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}