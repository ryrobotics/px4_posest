#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>
#include <mavros/mavros.h>
// #include <math_util.h>
// #include <frame_tf.h>

//msg head
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>

using namespace std;
// Data from MoCap
Eigen::Vector3d pos_drone_mocap;    //current position mocap
Eigen::Quaterniond q_mocap;         //current quaternion mocap
Eigen::Vector3d Euler_mocap;        //current attitude mocap
// Pos and Vel of Drone
Eigen::Vector3d pos_drone_fcu;      //current position FCU
Eigen::Vector3d vel_drone_fcu;      //current velocity FCU

Eigen::Quaterniond q_fcu;           //current quaternion FCU
Eigen::Vector3d Euler_fcu;          //current attitude FCU
// Publishment
geometry_msgs::PoseStamped vision;
// Func Statement
float get_dt(ros::Time last);
void printf_info();
// CallBack Func
void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Read the Drone Position from the Vrpn [Frame: mocap] (mocap->ENU Frame)
    Eigen::Vector3d pos_drone_mocap_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    pos_drone_mocap = pos_drone_mocap_enu;
    // Read the Drone Quaternion from the Vrpn, mocap is Z-up [Frame: mocap(ENU)]
    Eigen::Quaterniond q_mocap_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q_mocap = q_mocap_enu;

    // Tranfer the Quaternion to Euler Angles
    Euler_mocap = mavros::ftf::quaternion_to_rpy(q_mocap);
    
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    pos_drone_fcu = pos_drone_fcu_enu;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Read the Drone Velocity from the Mavros Package [Frame: ENU]
    Eigen::Vector3d vel_drone_fcu_enu(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);

    vel_drone_fcu = vel_drone_fcu_enu;
}

void euler_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Read the Drone Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    q_fcu = q_fcu_enu;

    // Transform the Quaternion to Euler Angles
    Euler_fcu = mavros::ftf::quaternion_to_rpy(q_fcu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_mocap");
    ros::NodeHandle nh("~");

    // Subscribe optitrack estimated position
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/rywang/pose", 1000, optitrack_cb);

    // Subscribe Drone's Position for Reference [Frame: ENU]
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // Subscribe Drone's Velocity for Reference [Frame: ENU]
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, vel_cb);

    // Subscribe Drone's Euler for Reference [Frame: ENU]
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, euler_cb);

    // Publish Drone's pose [Frame: ENU]
    // Send to FCU using mavros_extras/src/plugins/vision_pose_estimate.cpp, Mavlink Msg is VISION_POSITION_ESTIMATE
    // uORB msg in FCU is vehicle_vision_position.msg and vehicle_vision_attitude.msg
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);

    // Frequency
    ros::Rate rate(20.0);

    // Main Loop
    while (ros::ok())
    {
        // CallBack for Updating Sensor State
        ros::spinOnce();

        vision.pose.position.x = pos_drone_mocap[0];
        vision.pose.position.y = pos_drone_mocap[1];
        vision.pose.position.z = pos_drone_mocap[2];

        vision.pose.orientation.x = q_mocap.x();
        vision.pose.orientation.y = q_mocap.y();
        vision.pose.orientation.z = q_mocap.z();
        vision.pose.orientation.w = q_mocap.w();

        vision.header.stamp = ros::Time::now();
        vision_pub.publish(vision);

        // Print
        printf_info();
        rate.sleep();  
    }
    
    return 0;

}

float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void printf_info()
{
    ros::Time time_now = ros::Time::now();

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>PX4_POS_ESTIMATOR<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    // fixed point
    cout.setf(ios::fixed);
    // set precision
    cout<<setprecision(4);
    // left align
    cout.setf(ios::left);
    // show point
    cout.setf(ios::showpoint);
    // show pos
    cout.setf(ios::showpos);

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>mocap Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Pos_mocap [X Y Z] : " << pos_drone_mocap[0] << " [ m ] "<< pos_drone_mocap[1] <<" [ m ] "<< pos_drone_mocap[2] <<" [ m ] "<<endl;
    cout << "Euler_mocap [Yaw] : " << Euler_mocap[2] * 180/M_PI<<" [deg]  "<<endl;
        
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>FCU Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Pos_fcu [X Y Z] : " << pos_drone_fcu[0] << " [ m ] "<< pos_drone_fcu[1] <<" [ m ] "<< pos_drone_fcu[2] <<" [ m ] "<<endl;
    cout << "Vel_fcu [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "<< vel_drone_fcu[1] <<" [m/s] "<< vel_drone_fcu[2] <<" [m/s] "<<endl;
    cout << "Euler_fcu [Yaw] : " << Euler_fcu[2] * 180/M_PI<<" [deg] "<<endl;   
}
