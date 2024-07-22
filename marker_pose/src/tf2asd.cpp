#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
int main()
{
    tf2::Quaternion platform;
    platform.setW(0.707106);
    platform.setX(0.0);
    platform.setY(0.707106);
    platform.setZ(0.0);
    
    tf2::Quaternion drone;
    drone.setW(1);
    drone.setX(0.0);
    drone.setY(0.0);
    drone.setZ(0.0); 
    tf2::Quaternion ans = platform*drone.inverse();  // means you get quaternion sitting in platform frame to drone frame
    // double angle = acos(ans.getW())*2*180/3.14159;
    double angle = ans.getAngle()*180/3.14159;
    std::cout<<ans.getW()<<std::endl;
    std::cout<<ans.getX()<<std::endl;
    std::cout<<ans.getY()<<std::endl;
    std::cout<<ans.getZ()<<std::endl;
    std::cout<<angle<<std::endl;
    return 0;
}