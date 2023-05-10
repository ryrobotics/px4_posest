#include <ros/ros.h>
#include <px4_posest.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_posest_node");;
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    PX4_posest px4_posest(nh);

    // Main Loop
    while (ros::ok())
    {
        // CallBack for Updating Sensor State
        ros::spinOnce();
        px4_posest.printf_info();
        
        rate.sleep();
    }

    return 0;
}