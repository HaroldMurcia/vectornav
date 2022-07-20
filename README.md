# Vectornav300
This is my ros node for the vn-300 device from Vectornav.
The VN-300 is a miniature, high-performance Dual Antenna GNSS-Aided Inertial 
Navigation System that combines MEMS inertial sensors, two high-sensitivity 
GNSS receivers, and advanced Kalman filtering algorithms to provide optimal 
estimates of position, velocity, and orientation. By utilizing two separate 
GNSS receivers and antennas, the VN-300 enables accurate heading measurements 
without reliance on vehicle dynamics or magnetic  sensors, providing unmatched 
performance under both static and dynamic conditions.

![photo](/img/test-vn-300-rugged_large.png)

### Related info
* Official site: https://www.vectornav.com/products/detail/vn-300
* Datasheet: https://www.vectornav.com/resources/datasheets/vn-300-dual-gnss-ins
* Step FIle: https://www.vectornav.com/resources/cad-and-pcb-files
* Mechanical Drawing: https://www.vectornav.com/resources/cad-and-pcb-files

## How to install?

Install tf2 ROS library.
```
sudo apt-get install ros-kinetic-tf2-geometry-msgs
```

in your .../catkin_ws/src folder:

```
git clone https://github.com/HaroldMurcia/vectornav
cd ..
catkin_make
```

## How to run?

Define and include the installation parameters in the vn300.yaml file:
* serial_port
* serial_baud
* async_output_rate
* fixed_imu_rate
* frame_id
* Ecef2ENU
* tf_ned_to_enu
* wait_for_GNSS_startup
* linear_accel_covariance
* angular_vel_covariance
* orientation_covariance
* Antenna_A_offset
* baseline_position

```
sudo chmod 666 /dev/ttyUSB0
roslaunch vectornav vectornav.launch
```
Rostopic msg:
* /vectornav/GPS [NavSatFix](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/NavSatFix.html)
* /vectornav/GPSConStatus
  * |Header 
  * |float32 pvt_a
  * |float32 rtk_a
  * |float32 cn0_a
  * |float32 pvt_b
  * |float32 rtk_b
  * |float32 cn0_b
  * |float32 com_pvt
  * |float32 com_rtk
  * |uint8 conn_status_percent
  * |bool startup_complete
* /vectornav/IMU [Imu](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Imu.html)
* /vectornav/INSStatus
  * |Header
  * |uint8 ins_mode
  * |uint8 ins_gnss_fix
  * |uint8 ins_imu_error
  * |uint8 ins_mag_error
  * |uint8 ins_gnss_error
  * |uint8 ins_gnss_heading
  * |uint8 ins_gnss_compass
* /vectornav/INS_ECEF_Unc
* /vectornav/Mag [MagneticField](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/MagneticField.html)
* /vectornav/Odom [Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
* /vectornav/Pres [FluidPressure](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/FluidPressure.html)
* /vectornav/Temp [Temperature](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Temperature.html)
* /vectornav/TimeGpsPps
  * |Header header
  * |uint64 time_value
* /vectornav/TimeSyncIn
    * |Header header
    * |uint64 time_value
* /vectornav/pubTimeGps
    * |Header header
    * |uint64 time_value
