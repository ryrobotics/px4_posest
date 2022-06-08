# PX4_MoCap
Vicon or Nokov Opti Track System Test Code for Pixhawk with PX4

```shell
roscore

rosrun mavros mavros_node _fcu_url:=/dev/ttyUSB0:921600 _gcs_url:=udp://@172.16.254.1

rosrun px4_nokov px4_nokov_node

roslaunch vrpn_client_ros sample.launch server:=10.1.1.198

```
