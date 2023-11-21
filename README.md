# PX4_PoseEst

ROS nodes relaying Pose Estimation information from Vicon/T265/Lidar to PX4 /mavros/vision_pose/pose

```shell
roscore

roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600 gcs_url:=udp://@192.168.X.X

roslaunch vrpn_client_ros sample.launch server:=192.168.X.X

roslaunch px4_posest monitor.launch type:=0

sensor type:
    0 -> vicon
    1 -> T265
    2 -> Lidar
    3 -> Lidar_Imu_EKF
    4 -> VINS_Imu_EKF

is_pub:
    0 -> no publisher
    1 -> publish odom to /mavros/vision_pose/pose

```
