#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>
#include <mavros/mavros.h>
// #include <math_util.h>
// #include <frame_tf.h>

// msg head
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
#include <geometry_msgs/PoseWithCovariance.h>

using namespace std;
using namespace Eigen;

int sensor_type;

// Data from MoCap
Vector3d pos_drone_mocap; // current position mocap
Quaterniond q_mocap;      // current quaternion mocap
Vector3d Euler_mocap;     // current attitude mocap

// Data from T265
Vector3d pos_drone_t265;
Quaterniond q_t265;
Vector3d Euler_t265;

// Data from os0
Vector3d pos_drone_os0;
Quaterniond q_os0;
Vector3d Euler_os0;

// Pos and Vel of Drone
Vector3d pos_drone_fcu; // current position FCU
Vector3d vel_drone_fcu; // current velocity FCU

Quaterniond q_fcu;  // current quaternion FCU
Vector3d Euler_fcu; // current attitude FCU

// Publishment
geometry_msgs::PoseStamped vision;

// Func Statement
float get_dt(ros::Time last);
void printf_info();

// CallBack Func

void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Vrpn [Frame: mocap] (mocap->ENU Frame)
    Vector3d pos_drone_mocap_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    pos_drone_mocap = pos_drone_mocap_enu;
    // Read the Drone Quaternion from the Vrpn, mocap is Z-up [Frame: mocap(ENU)]
    Quaterniond q_mocap_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q_mocap = q_mocap_enu;

    // Tranfer the Quaternion to Euler Angles
    Euler_mocap = mavros::ftf::quaternion_to_rpy(q_mocap);
}

void vio_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Read the Drone Position from the Vrpn [Frame: mocap] (mocap->ENU Frame)
    Vector3d pos_drone_t265_enu(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    pos_drone_t265 = pos_drone_t265_enu;
    // Read the Drone Quaternion from the Vrpn, mocap is Z-up [Frame: mocap(ENU)]
    Quaterniond q_t265_enu(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    q_t265 = q_t265_enu;

    // Tranfer the Quaternion to Euler Angles
    Euler_t265 = mavros::ftf::quaternion_to_rpy(q_t265);
}

void lio_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Read the Drone Position from the 3D Lidar [Frame: ENU] (OS0->ENU Frame)
    Vector3d pos_drone_os0_enu(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    pos_drone_os0 = pos_drone_os0_enu;
    // Read the Drone Quaternion from the 3D Lidar, os0 is Z-up [Frame: ENU]
    Quaterniond q_os0_enu(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    q_os0 = q_os0_enu;

    // Tranfer the Quaternion to Euler Angles
    Euler_os0 = mavros::ftf::quaternion_to_rpy(q_os0);
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Vector3d pos_drone_fcu_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    pos_drone_fcu = pos_drone_fcu_enu;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Read the Drone Velocity from the Mavros Package [Frame: ENU]
    Vector3d vel_drone_fcu_enu(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

    vel_drone_fcu = vel_drone_fcu_enu;
}

void euler_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    // Read the Drone Quaternion from the Mavros Package [Frame: ENU]
    Quaterniond q_fcu_enu(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    q_fcu = q_fcu_enu;

    // Transform the Quaternion to Euler Angles
    Euler_fcu = mavros::ftf::quaternion_to_rpy(q_fcu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_mix_monitor");
    ros::NodeHandle nh("~");

    nh.param<int>("sensor_type", sensor_type, 0); // 0->vicon, 1->lidar, 2->vio

    // Subscribe optitrack estimated position
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/rywang/pose", 1000, optitrack_cb);

    // Subscribe t265 odom
    ros::Subscriber vio_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample_throttled", 10, vio_cb);

    // Subscribe os0 odom
    ros::Subscriber lio_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 10, lio_cb);

    // Subscribe Drone's Position for Reference [Frame: ENU]
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // Subscribe Drone's Velocity for Reference [Frame: ENU]
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, vel_cb);

    // Subscribe Drone's Euler for Reference [Frame: ENU]
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, euler_cb);

    // Publish Drone's pose [Frame: ENU]
    // Send to FCU using mavros_extras/src/plugins/vision_pose_estimate.cpp, Mavlink Msg is VISION_POSITION_ESTIMATE
    // uORB msg in FCU is vehicle_vision_position.msg and vehicle_vision_attitude.msg
    ros::Publisher vision_pub;
    if (sensor_type != 2) // if using t265, no need publish pose
    {
        vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);
    }

    // Frequency
    ros::Rate rate(20.0);

    // Main Loop
    while (ros::ok())
    {
        // CallBack for Updating Sensor State
        ros::spinOnce();

        if (sensor_type == 0)
        {
            vision.pose.position.x = pos_drone_mocap[0];
            vision.pose.position.y = pos_drone_mocap[1];
            vision.pose.position.z = pos_drone_mocap[2];

            vision.pose.orientation.x = q_mocap.x();
            vision.pose.orientation.y = q_mocap.y();
            vision.pose.orientation.z = q_mocap.z();
            vision.pose.orientation.w = q_mocap.w();

            vision.header.stamp = ros::Time::now();
            vision_pub.publish(vision);
        }
        else if (sensor_type == 1)
        {
            vision.pose.position.x = pos_drone_os0[0];
            vision.pose.position.y = pos_drone_os0[1];
            vision.pose.position.z = pos_drone_os0[2];

            vision.pose.orientation.x = q_os0.x();
            vision.pose.orientation.y = q_os0.y();
            vision.pose.orientation.z = q_os0.z();
            vision.pose.orientation.w = q_os0.w();

            vision.header.stamp = ros::Time::now();
            vision_pub.publish(vision);
        }

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

    cout << ">>>>>>>>>>>>>>>>>>>>PX4_POS_ESTIMATOR<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;

    if (sensor_type == 0)
        cout << ">>>>>>>>>>>>>>>>>>>>Pos Data from Vicon<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    else if (sensor_type == 1)
        cout << ">>>>>>>>>>>>>>>>>>>>Pos Data from Lidar<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    else if (sensor_type == 2)
        cout << ">>>>>>>>>>>>>>>>>>>>Pos Data from T265<<<<<<<<<<<<<<<<<<<<<<<<" << endl;

    // fixed point
    cout.setf(ios::fixed);
    // set precision
    cout << setprecision(4);
    // left align
    cout.setf(ios::left);
    // show point
    cout.setf(ios::showpoint);
    // show pos
    cout.setf(ios::showpos);

    cout << ">>>>>>>>>>>>>>>>>>>>Mocap Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "Pos_mocap [X Y Z] : " << pos_drone_mocap[0] << " [ m ] " << pos_drone_mocap[1] << " [ m ] " << pos_drone_mocap[2] << " [ m ] " << endl;
    cout << "Euler_mocap [Yaw] : " << Euler_mocap[2] * 180 / M_PI << " [deg]  " << endl;

    cout << ">>>>>>>>>>>>>>>>>>>>T265 Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "Pos_t265 [X Y Z] : " << pos_drone_t265[0] << " [ m ] " << pos_drone_t265[1] << " [ m ] " << pos_drone_t265[2] << " [ m ] " << endl;
    cout << "Euler_t265 [Yaw] : " << Euler_t265[2] * 180 / M_PI << " [deg]  " << endl;

    cout << ">>>>>>>>>>>>>>>>>>>>OS0 Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "Pos_os0 [X Y Z] : " << pos_drone_os0[0] << " [ m ] " << pos_drone_os0[1] << " [ m ] " << pos_drone_os0[2] << " [ m ] " << endl;
    cout << "Euler_os0 [Yaw] : " << Euler_os0[2] * 180 / M_PI << " [deg]  " << endl;

    cout << ">>>>>>>>>>>>>>>>>>>>FCU Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "Pos_fcu [X Y Z] : " << pos_drone_fcu[0] << " [ m ] " << pos_drone_fcu[1] << " [ m ] " << pos_drone_fcu[2] << " [ m ] " << endl;
    cout << "Vel_fcu [X Y Z] : " << vel_drone_fcu[0] << " [m/s] " << vel_drone_fcu[1] << " [m/s] " << vel_drone_fcu[2] << " [m/s] " << endl;
    cout << "Euler_fcu [Yaw] : " << Euler_fcu[2] * 180 / M_PI << " [deg] " << endl;
}
