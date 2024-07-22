#include <marker_pose/io_pose_header.hpp>
#include <marker_pose/ros_headers.hpp>
// #include <tf2_ros/buffer.h>

tf::StampedTransform RobotTransform1;
tf::StampedTransform RobotTransform2;
tf::StampedTransform RobotTransform3;
tf::StampedTransform MarkerTransform;

void write_marker_pose(std::string id, tf2::Vector3 xyzMarker)
{
    std::ofstream outputFile("marker_pose/src/marker_pose.txt",std::ios::app);
    std::cout << xyzMarker.x() << std::endl;

    if(outputFile.is_open())
    {
        outputFile << id << "," << xyzMarker.x() << "," << xyzMarker.y() << "," << xyzMarker.z() << std::endl; // Write to file
        outputFile.close(); // Close the file
        std::cout << "----Data-written-to-the-file-successfully----" << std::endl;
    } 
    else 
    {
        std::cout << "Failed to open the file for writing." << std::endl;
    }
    return;
}



bool existsID(std::string id)
{
    std::ifstream inputFile("marker_pose/src/marker_pose.txt");
    bool ans;
    if(!inputFile.is_open())
    {
        std::cout << "Failed to open the file." << std::endl;
        return false; 
    }
    else
    {
        std::string wordPattern = id;
        std::string line;
        while (std::getline(inputFile, line)) 
        {
            // Check if the word pattern exists in the line
            if (line.find(wordPattern) != std::string::npos) 
            {
                ans = true;
                break;      // Found the word and hence break out of the while loop            
            }
            else
                ans = false;
        }
        inputFile.close();
        return ans;
    }
}


void pose_calc(std::string id, tf::StampedTransform RobotTransform1, tf::StampedTransform RobotTransform2,tf::StampedTransform RobotTransform3,tf::StampedTransform MarkerTransform)
{
    tf2::Quaternion qRobot1(RobotTransform1.getRotation().w(), RobotTransform1.getRotation().x(), RobotTransform1.getRotation().y(), RobotTransform1.getRotation().z());
    tf2::Matrix3x3 rotRobot1(qRobot1);
    tf2::Vector3 originRobot1(RobotTransform1.getOrigin().x(), RobotTransform1.getOrigin().y(), RobotTransform1.getOrigin().z());

    tf2::Quaternion qRobot2(RobotTransform2.getRotation().w(), RobotTransform2.getRotation().x(), RobotTransform2.getRotation().y(), RobotTransform2.getRotation().z());
    tf2::Matrix3x3 rotRobot2(qRobot2);
    tf2::Vector3 originRobot2(RobotTransform2.getOrigin().x(), RobotTransform2.getOrigin().y(), RobotTransform2.getOrigin().z());

    tf2::Quaternion qRobot3(RobotTransform3.getRotation().w(), RobotTransform3.getRotation().x(), RobotTransform3.getRotation().y(), RobotTransform3.getRotation().z());
    tf2::Matrix3x3 rotRobot3(qRobot3);
    tf2::Vector3 originRobot3(RobotTransform3.getOrigin().x(), RobotTransform3.getOrigin().y(), RobotTransform3.getOrigin().z());

    tf2::Quaternion qMarker(MarkerTransform.getRotation().w(), MarkerTransform.getRotation().x(), MarkerTransform.getRotation().y(), MarkerTransform.getRotation().z());
    tf2::Matrix3x3 rotMarker(qMarker);
    tf2::Vector3 originMarker(MarkerTransform.getOrigin().x(), MarkerTransform.getOrigin().y(), MarkerTransform.getOrigin().z());

    tf2::Vector3 xyzMarker =  originRobot1 + rotRobot1 * (originRobot2 + rotRobot2 * (originRobot3 + rotRobot3 * originMarker));
    // tf2::Matrix3x3 rpyMarker = rotRobot * rotMarker;
    // ROS_INFO("all good till here...");
    if(!existsID(id))
        write_marker_pose(id,xyzMarker);

    return;
}
void transformCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    if(msg->transforms.size() == 0)
    {
        ROS_INFO("No tf received...");
        return;
    }
    // else
    // {
    //     std::cout << msg->transforms[0] << std::endl;
    //     return;
    // }
    // ROS_INFO("Entered CallBack...");
    std::string names[] = {"t0","t1","t2","t3","t4"};   
    std::string id;
    // ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
    bool init = false ;

    // tf2_ros::Buffer listener;
    // tf2_ros::TransformListener buf(listener);
    // listener.waitForTransform("t0", "base_link", ros::Time(0), ros::Duration(10.0));
    // ROS_INFO("Total tf captured: [%d]", tfMessage.transforms.size());   
    // tf::TransformListener::lookupTransform("t0", "base_link", ros::Time(0), transform);

    if(msg->transforms[0].child_frame_id == "jetauto_1/odom" && msg->transforms[0].header.frame_id == "jetauto_1/map")
    {
        RobotTransform1.setRotation(tf::Quaternion(msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z, msg->transforms[0].transform.rotation.w));
        RobotTransform1.setOrigin(tf::Vector3(msg->transforms[0].transform.translation.x, msg->transforms[0].transform.translation.y, msg->transforms[0].transform.translation.z));
        RobotTransform1.child_frame_id_ = msg->transforms[0].child_frame_id;
        RobotTransform1.frame_id_ = msg->transforms[0].header.frame_id;
    }
    else if(msg->transforms[0].child_frame_id == "jetauto_1/base_footprint" && msg->transforms[0].header.frame_id == "jetauto_1/odom")
    {
        RobotTransform2.setRotation(tf::Quaternion(msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z, msg->transforms[0].transform.rotation.w));
        RobotTransform2.setOrigin(tf::Vector3(msg->transforms[0].transform.translation.x, msg->transforms[0].transform.translation.y, msg->transforms[0].transform.translation.z));
        RobotTransform2.child_frame_id_ = msg->transforms[0].child_frame_id;
        RobotTransform2.frame_id_ = msg->transforms[0].header.frame_id;
    }
    else if(msg->transforms[0].child_frame_id == "jetauto_1/camera_link" && msg->transforms[0].header.frame_id == "jetauto_1/base_link")
    {
        RobotTransform3.setRotation(tf::Quaternion(msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z, msg->transforms[0].transform.rotation.w));
        RobotTransform3.setOrigin(tf::Vector3(msg->transforms[0].transform.translation.x, msg->transforms[0].transform.translation.y, msg->transforms[0].transform.translation.z));
        RobotTransform3.child_frame_id_ = msg->transforms[0].child_frame_id;
        RobotTransform3.frame_id_ = msg->transforms[0].header.frame_id;
    }
    else
    {
        // std::cout << "entered"<<std::endl;
        for(auto i : names)
        {
            if(msg->transforms[0].child_frame_id == i && msg->transforms[0].header.frame_id == "jetauto_1/camera_rgb_optical_frame")
            {
                // ROS_INFO("Marker Transform Detected...");
                init = true;
                id=i;
                MarkerTransform.setRotation(tf::Quaternion(msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z, msg->transforms[0].transform.rotation.w));
                MarkerTransform.setOrigin(tf::Vector3(msg->transforms[0].transform.translation.x, msg->transforms[0].transform.translation.y, msg->transforms[0].transform.translation.z));
                MarkerTransform.child_frame_id_ = msg->transforms[0].child_frame_id;
                MarkerTransform.frame_id_ = msg->transforms[0].header.frame_id;
                break;
            }
        }
        if(init)
            pose_calc(id,RobotTransform1,RobotTransform2,RobotTransform3,MarkerTransform);
        // ROS_INFO("x: [%f], y: [%f], z: [%f]", i.transform.translation.x, i.transform.translation.y, i.transform.translation.z);
        // std::cout << i << std::endl;

        // if(listener.canTransform("camera_rgb_optical_frame","t1", ros::Time(0)))
        // {
        //     ROS_INFO("Found Transform...");
        //     init = true;
        //     id=i;
        //     // listener.lookupTransform(i, "camera_rgb_optical_frame", ros::Time(0), MarkerTransform);
        //     // listener.lookupTransform("base_link", "map", ros::Time(0), RobotTransform);
        //     break;
        // }
    // std::cout << MarkerTransform.getOrigin().x() << std::endl;
    }
    return;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"marker_pose");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe("/tf",1,&transformCallback);
    ros::Rate r(30);
    // ROS_INFO("Starting Shit!!...");
    std::ofstream ofs;
    ofs.open("marker_pose/src/marker_pose.txt", std::ofstream::out | std::ofstream::trunc);
    ofs.close();
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}