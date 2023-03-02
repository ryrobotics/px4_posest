#include <ros/ros.h>
#include <px4_posest.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_posest_node");;
    ros::NodeHandle nh;
    ros::Rate rate(50.0);

    PX4_posest px4_posest(nh);

    px4_posest.camera_initial();
    
    ros::Time time_now = ros::Time::now();
    ros::Time time_lst = ros::Time::now();
    time_lst.sec = time_lst.sec - 10;

    // Main Loop
    while (ros::ok())
    {
        // CallBack for Updating Sensor State
        ros::spinOnce();

        time_now = ros::Time::now();

        if((time_now - time_lst).toSec() > 0.2)
        {
            px4_posest.printf_info();
            time_lst = time_now;
        }
        
        rate.sleep();
    }

    return 0;
}