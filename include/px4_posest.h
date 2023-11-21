#ifndef PX4_POSEST_H
#define PX4_POSEST_H


#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <mavros/mavros.h>
// #include <math_util.h>
// #include <frame_tf.h>

// msg head
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <tf2_msgs/TFMessage.h>
#include <serial/serial.h>

using namespace std;
using namespace Eigen;

class PX4_posest {
  public:
    PX4_posest(ros::NodeHandle& nh);
    // ~PX4_posest();

    ros::Subscriber mocap_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber ekf_sub;
    ros::Subscriber position_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber attitude_sub;
    ros::Subscriber batt_sub;
    ros::Subscriber rng_sub;

    ros::Publisher vision_pub;

    ros::Timer timer_vision_pub;
    ros::Timer timer_take_photo;

    int sensor_type;
    bool is_pub;
    bool is_print;

    // 0->vicon, 1->vio, 2->lidar, 3->imu_lidar_ekf
    enum SENSOR_TYPE
    {
        MOCAP = 0,
        VIO = 1,
        LIO = 2,
        LIO_EKF = 3,
        VINS_EKF = 4
    };

    double voltage, percentage, range;
    bool camera_flag;
    int camera_cnt;

    Eigen::Vector3d euler_fcu;
    Eigen::Vector3d euler_odom;
    Eigen::Vector3d euler_ekf;
    Eigen::Vector3d euler_mocap;

    Vector3d px4_pose;
    Vector3d px4_vel;
    geometry_msgs::PoseStamped mocap_pose;
    geometry_msgs::PoseStamped odom_pose;
    geometry_msgs::PoseStamped ekf_pose;
    geometry_msgs::PoseStamped vision_pose;

    nav_msgs::Odometry odom_rcv;
    ros::Time odom_rcv_stamp;
    ros::Time ekf_rcv_stamp;

    serial::Serial ser;
    
    void printf_info();
    void camera_initial();

  private:
    float get_dt(ros::Time last);
    void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void ekf_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void att_cb(const sensor_msgs::Imu::ConstPtr &msg);
    void batt_cb(const sensor_msgs::BatteryState::ConstPtr &msg);
    void rng_cb(const sensor_msgs::Range::ConstPtr &msg);
    void timercb_pub_vision_pose(const ros::TimerEvent &e);
    void timercb_take_photo(const ros::TimerEvent &e);
    void writeToFile(const std::string& data);

    inline bool odom_is_received(const ros::Time &now_time)
    {
      return (now_time - odom_rcv_stamp).toSec() < 0.5;
    }

    inline bool ekf_is_received(const ros::Time &now_time)
    {
      return (now_time - ekf_rcv_stamp).toSec() < 0.5;
    }

    inline bool odom_is_good(const nav_msgs::Odometry &msg)
    {
      Eigen::Vector3d v;
      v(0) = msg.twist.twist.linear.x;
      v(1) = msg.twist.twist.linear.y;
      v(2) = msg.twist.twist.linear.z;

      Eigen::Vector3d p;
      p(0) = msg.pose.pose.position.x;
      p(1) = msg.pose.pose.position.y;
      p(2) = msg.pose.pose.position.z;

      return v.norm() < 3.0 && p.norm() < 10.0;
    }
};

#endif