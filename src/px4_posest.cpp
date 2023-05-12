#include "px4_posest.h"

PX4_posest::PX4_posest(ros::NodeHandle &nh)
{
    nh.param<int>("px4_posest_node/sensor_type", sensor_type, SENSOR_TYPE::MOCAP); // 0->vicon, 1->vio, 2->lio, 3->imu_lidar_ekf
    
    if(sensor_type == SENSOR_TYPE::MOCAP)
    {
        // Subscribe mocap estimated position
        mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/rywang/pose", 1000, &PX4_posest::mocap_cb, this);
    }
    else if(sensor_type == SENSOR_TYPE::VIO)
    {
        // Subscribe t265 odom
        vio_sub = nh.subscribe<nav_msgs::Odometry>("camera/odom/sample_throttled", 10, &PX4_posest::vio_cb, this);
    }
    else if(sensor_type == SENSOR_TYPE::LIO)
    {
        // Subscribe os0 odom
        lio_sub = nh.subscribe<nav_msgs::Odometry>("Odometry", 10, &PX4_posest::lio_cb, this);        
    }
    else if(sensor_type == SENSOR_TYPE::EKF)
    {
        // Subscribe os0 odom
        lio_sub = nh.subscribe<nav_msgs::Odometry>("Odometry", 10, &PX4_posest::lio_cb, this);        
        // Subscribe imu_ekf odom
        ekf_sub = nh.subscribe<nav_msgs::Odometry>("imu_ekf/odom", 10, &PX4_posest::ekf_cb, this);        
    }

    // Subscribe Drone's Position for Reference [Frame: ENU]
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &PX4_posest::pos_cb, this);

    // Subscribe Drone's Velocity for Reference [Frame: ENU]
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, &PX4_posest::vel_cb, this);

    // Subscribe Drone's Euler for Reference [Frame: ENU]
    attitude_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &PX4_posest::att_cb, this);

    batt_sub = nh.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10, &PX4_posest::batt_cb, this);    
    
    // rng_sub = nh.subscribe<sensor_msgs::Range>("mavros/distance_sensor/hrlv_ez4_pub", 10, &PX4_posest::rng_cb, this);

    // Publish Drone's pose [Frame: ENU]
    // Send to FCU using mavros_extras/src/plugins/vision_pose_estimate.cpp, Mavlink Msg is VISION_POSITION_ESTIMATE
    // uORB msg in FCU is vehicle_vision_position.msg and vehicle_vision_attitude.msg
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);

    if (sensor_type != SENSOR_TYPE::VIO) // if using t265, no need publish pose
    {
        timer_vision_pub = nh.createTimer(ros::Duration(0.02), &PX4_posest::timercb_pub_vision_pose, this);
    }

    timer_take_photo = nh.createTimer(ros::Duration(2), &PX4_posest::timercb_take_photo, this);
}

// CallBack Func

void PX4_posest::timercb_pub_vision_pose(const ros::TimerEvent &e)
{
    if (sensor_type == SENSOR_TYPE::MOCAP)
    {
        vision_pose = mocap_pose;
    }
    else if(sensor_type == SENSOR_TYPE::LIO)
    {
        vision_pose = lio_pose;
    }
    else if(sensor_type == SENSOR_TYPE::EKF)
    {
        vision_pose = ekf_pose;
    }

    vision_pose.header.stamp = ros::Time::now();
    vision_pub.publish(vision_pose);
}

void PX4_posest::camera_initial()
{
    ser.setPort("/dev/ttyTHS0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(200);
    ser.setTimeout(to);
    ser.open();
    ser.flushInput();
    ser.flushOutput();
    camera_flag = false;
    camera_cnt = -1;
    writeToFile("\nNEW Recording\n");
}

void PX4_posest::timercb_take_photo(const ros::TimerEvent &e)
{
    // Send the command bytes
    std::vector<uint8_t> command_bytes = {0xaa, 0x05, 0x01, 0x04, 0x0a};
    ser.write(command_bytes);

    // Wait for and read the response bytes
    std::vector<uint8_t> response_bytes;
    response_bytes.resize(6);
    ser.read(response_bytes.data(), 6);

    // Check if the response bytes match the expected bytes
    std::vector<uint8_t> expected_response_bytes = {0x55, 0x06, 0x01, 0x00, 0x04, 0xe2};

    if (response_bytes == expected_response_bytes)
    {
        camera_flag = true;
        camera_cnt++;

        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2);

        stream << "Cnt: " << camera_cnt
               << " Pos_x: " << px4_pose[0]
               << " Pos_y: " << px4_pose[1]
               << " Pos_z: " << px4_pose[2];
        
        writeToFile(stream.str());
    }
    else 
        camera_flag = false;
}

void PX4_posest::writeToFile(const string& data)
{    
    string full_path = "/src/archsd_photo.txt";

    // Open the file for writing
    ofstream outfile(full_path.c_str(), std::ios::app);

    if (outfile.is_open())
    {
        outfile << data << std::endl;
        // Close the file
        outfile.close();
    } 
    else
        ROS_ERROR_STREAM("Unable to open file: " << full_path);
}

void PX4_posest::mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    mocap_pose = *msg;
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
    euler_mocap = mavros::ftf::quaternion_to_rpy(q);
}

void PX4_posest::vio_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    t265_pose.header = msg->header;
    t265_pose.pose = msg->pose.pose;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    euler_vio = mavros::ftf::quaternion_to_rpy(q);
}

void PX4_posest::lio_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    lio_pose.header = msg->header;
    lio_pose.pose = msg->pose.pose;
    
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    euler_lio = mavros::ftf::quaternion_to_rpy(q);
}

void PX4_posest::ekf_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    ekf_pose.header = msg->header;
    ekf_pose.pose = msg->pose.pose;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    euler_ekf = mavros::ftf::quaternion_to_rpy(q);
}

void PX4_posest::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    px4_pose[0] = msg->pose.position.x;
    px4_pose[1] = msg->pose.position.y;
    px4_pose[2] = msg->pose.position.z;
}

void PX4_posest::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    px4_vel[0] = msg->twist.linear.x;
    px4_vel[1] = msg->twist.linear.y;
    px4_vel[2] = msg->twist.linear.z;
}

void PX4_posest::att_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    // Read the Drone Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    // Transform the Quaternion to Euler Angles
    euler_fcu = mavros::ftf::quaternion_to_rpy(q_fcu);
}

void PX4_posest::batt_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    voltage = msg->voltage;
    percentage = msg->percentage;
}

// If use this module, pls comment the '- distance_sensor' line, which is in the default plugin_blacklist
// The file location is /opt/ros/melodic/share/mavros/launch/px4_pluginlists.yaml
// void PX4_posest::rng_cb(const sensor_msgs::Range::ConstPtr &msg)
// {
//     range = msg->range;
// }


float PX4_posest::get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void PX4_posest::printf_info()
{
    ros::Time time_now = ros::Time::now();

    // cout << ">>>>>>>>>>>>>>>>>>>>PX4_POS_ESTIMATOR<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;

    // fixed point
    cout.setf(ios::fixed);
    // set precision
    cout << setprecision(2);
    // left align
    cout.setf(ios::left);
    // show point
    cout.setf(ios::showpoint);
    // show pos
    cout.setf(ios::showpos);

    if (sensor_type == SENSOR_TYPE::MOCAP)
    {
        cout << ">>>Data from Vicon<<<" << endl;
        cout << ">>>Mocap Info<<<" << endl;
        cout << "Pos_mocap: " << mocap_pose.pose.position.x << " [m] " << mocap_pose.pose.position.y << " [m] " << mocap_pose.pose.position.z << " [m] " << endl;
        cout << "Euler_mocap [Yaw] : " << euler_mocap[2] * 180 / M_PI << " [deg]  " << endl;
    }
    else if (sensor_type == SENSOR_TYPE::VIO)
    {
        cout << ">>>Data from T265<<<" << endl;
        cout << ">>>T265 Info<<<" << endl;
        cout << "Pos_t265: " << t265_pose.pose.position.x << " [m] " << t265_pose.pose.position.y << " [m] " << t265_pose.pose.position.z << " [m] " << endl;
        cout << "Euler_t265 [Yaw] : " << euler_vio[2] * 180 / M_PI << " [deg]  " << endl;        
    }
    else if (sensor_type == SENSOR_TYPE::LIO)
    {
        cout << ">>>Data from Lidar<<<" << endl;
        cout << ">>>FAST_LIO Info<<<" << endl;
        cout << "Pos_LIO: " << lio_pose.pose.position.x << " [m] " << lio_pose.pose.position.y << " [m] " << lio_pose.pose.position.z << " [m] " << endl;
        cout << "Euler_LIO [Yaw] : " << euler_lio[2] * 180 / M_PI << " [deg]  " << endl;
    }
    else if (sensor_type == SENSOR_TYPE::EKF)
    {
        cout << ">>>Data from IMU_EKF<<<" << endl;
        cout << ">>>FAST_LIO Info<<<" << endl;
        cout << "Pos_LIO: " << lio_pose.pose.position.x << " [m] " << lio_pose.pose.position.y << " [m] " << lio_pose.pose.position.z << " [m] " << endl;
        cout << "Euler_LIO [Yaw] : " << euler_lio[2] * 180 / M_PI << " [deg]  " << endl;

        cout << ">>>IMU_EKF Info<<<" << endl;
        cout << "Pos_ekf: " << ekf_pose.pose.position.x << " [m] " << ekf_pose.pose.position.y << " [m] " << ekf_pose.pose.position.z << " [m] " << endl;
        cout << "Euler_ekf [Yaw] : " << euler_ekf[2] * 180 / M_PI << " [deg]  " << endl;
    }


    cout << ">>>FCU Info<<<" << endl;
    cout << "Pos_px4: " << px4_pose[0] << " [m] " << px4_pose[1] << " [m] " << px4_pose[2] << " [m] " << endl;
    // cout << "Vel_fcu: " << px4_vel[0] << " [m/s] " << px4_vel[1] << " [m/s] " << px4_vel[2] << " [m/s] " << endl;
    cout << "Euler_px4 [Yaw] : " << euler_fcu[2] * 180 / M_PI << " [deg] " << endl;
    cout << "Batt : " << voltage << " [V] " << percentage * 100 << "%" << endl;
    cout << "Camera: " << camera_cnt << " [" << camera_flag << "]" << endl;
    // cout << "UP_Dist : " << range << " [m] " << "Camera: "<< camera_cnt << " [" << camera_flag << "]"<< endl;
}
