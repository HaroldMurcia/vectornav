#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

// ROS Libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "diagnostic_msgs/KeyValue.h"
#include "std_srvs/Empty.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/util.h"
#include "vn/compositedata.h"
#include "vn/matrix.h"
// We need this file for our sleep function.
#include "vn/thread.h"

// Own MSGs
#include "vectornav/vn_time.h"
#include "vectornav/ins_status.h"
#include "vectornav/gps_conn_status.h"
#include "vectornav/ecef_uncert.h"


ros::Publisher pubIMU, pubMag, pubGPS, pubOdom, pubTemp, pubPres, pubGPSConStatus, pubTimeSyncIn, pubTimeGps, pubTimeGpsPps, pubINSstatus, pubECEFun;
ros::ServiceServer resetOdomSrv;

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Custom user data to pass to packet callback function
struct UserData{
    int device_family;
};


//Unused covariances initilized to zero's
boost::array<double, 9ul> linear_accel_covariance = { };
boost::array<double, 9ul> angular_vel_covariance = { };
boost::array<double, 9ul> orientation_covariance = { };
vec3f Antenna_A_offset;
vec3f baseline_position;
XmlRpc::XmlRpcValue rpc_temp;

// Global Variables
bool flag_origin = 0;     // Flag to indicate the origin position
bool flag_connecting = 1; // Flag to indicate the first time of execution
bool flag_startup_header_printed = false; // Flag to avoid overprint in console
bool ENU_flag;
bool tf_ned_to_enu;
bool frame_based_enu;
bool wait_for_GNSS_startup;
int Perc = 0;
vec3d pos_o;

std::string frame_id;

vectornav::gps_conn_status _connStatus;


// ......................................................................................................FUNCTIONS

void print_progress_bar(int percentage){
    string progress = "[" + string(percentage, '|') + string(100 - percentage, ' ') + "]";
    cout << progress << "\r\033[F\033[F\033[F" << flush;
}

void ClearCharArray(char *Data, int length){
    for (int i = 0; i < length; i++){
        Data[i] = 0;
    }
}

void ConnectionState(void *userData, const char *rawData, size_t length, size_t runningIndex){
    // cout << rawData << endl;
    char header1[] = "$VNRRG,98,"; // header of GNSS Compass Startup Status
    char header2[] = "$VNRRG,86,"; // header of GNSS Compass Signal Health Status
    int j = 0;
    //cout << rawData << endl;
    // Decode register 98
    _connStatus.header.stamp = ros::Time::now();
    for (int i = 0; i < length; i++)
    {
        if ((rawData[i] == header1[j]))
        {
            j++;
            if (j == 10)
            {
                char Perc_char[3] = {rawData[i + 1], rawData[i + 2], rawData[i + 3]};
                Perc = atoi(Perc_char);
                _connStatus.conn_status_percent = Perc;
                i = length;
                if (Perc == 100)
                {
                    flag_connecting = 0;
                }
            }
        }
        else
        {
            j = 0;
        }
    }

    // Decode register 86
    float PVT_A = 0, RTK_A = 0, CN0_A = 0, PVT_B = 0, RTK_B = 0, CN0_B = 0, ComPVT = 0, ComRTK = 0;
    j = 0;
    char aux[5];  // Save part of data of the message
    int aux1;     // Save the actual index
    int flag3 = 0;
    ClearCharArray(aux, 5);
    for (int i = 0; i < length; i++)
    {
        if ((rawData[i] == header2[j]))
        {
            j++;
            if (j == 10)
            {
                //cout << *rawData.substr(i,i+10) << endl;
                i++;
                for (int k = 0; k < 5; k++)
                {
                    // cout << "flag3: " << flag3 << endl;
                    // PVT_A
                    if ((rawData[i + k] != ',') && (flag3 == 0))
                    {
                        //cout << rawData[i+k] << endl;
                        aux[k] = rawData[i + k];
                    }
                    else if ((rawData[i + k] == ',') && (flag3 == 0))
                    {
                        PVT_A = atof(aux);
                        //cout << aux << endl;
                        ClearCharArray(aux, 5);
                        //cout << PVT_A << endl;
                        flag3++;
                        i += (k + 1);
                        k = 0;
                    }
                    // RTK_A
                    if ((rawData[i + k] != ',') && (flag3 == 1))
                    {
                        //cout << rawData[i+k] << endl;
                        aux[k] = rawData[i + k];
                    }
                    else if ((rawData[i + k] == ',') && (flag3 == 1))
                    {
                        RTK_A = atof(aux);
                        //cout << aux << endl;
                        ClearCharArray(aux, 5);
                        //cout << RTK_A << endl;
                        flag3++;
                        i += (k + 1);
                        k = 0;
                    }
                    // CN0_A
                    if ((rawData[i + k] != ',') && (flag3 == 2))
                    {
                        //cout << rawData[i+k] << endl;
                        aux[k] = rawData[i + k];
                    }
                    else if ((rawData[i + k] == ',') && (flag3 == 2))
                    {
                        CN0_A = atof(aux);
                        //cout << aux << endl;
                        ClearCharArray(aux, 5);
                        //cout << CN0_A << endl;
                        flag3++;
                        i += (k + 1);
                        k = 0;
                    }
                    // PVT_B
                    if ((rawData[i + k] != ',') && (flag3 == 3))
                    {
                        //cout << rawData[i+k] << endl;
                        aux[k] = rawData[i + k];
                    }
                    else if ((rawData[i + k] == ',') && (flag3 == 3))
                    {
                        PVT_B = atof(aux);
                        //cout << aux << endl;
                        ClearCharArray(aux, 5);
                        //cout << PVT_B << endl;
                        flag3++;
                        i += (k + 1);
                        k = 0;
                    }
                    // RTK_B
                    if ((rawData[i + k] != ',') && (flag3 == 4))
                    {
                        //cout << rawData[i+k] << endl;
                        aux[k] = rawData[i + k];
                    }
                    else if ((rawData[i + k] == ',') && (flag3 == 4))
                    {
                        RTK_B = atof(aux);
                        //cout << aux << endl;
                        ClearCharArray(aux, 5);
                        //cout << RTK_B << endl;
                        flag3++;
                        i += (k + 1);
                        k = 0;
                    }
                    // CN0_B
                    if ((rawData[i + k] != ',') && (flag3 == 5))
                    {
                        //cout << rawData[i+k] << endl;
                        aux[k] = rawData[i + k];
                    }
                    else if ((rawData[i + k] == ',') && (flag3 == 5))
                    {
                        CN0_B = atof(aux);
                        //cout << aux << endl;
                        ClearCharArray(aux, 5);
                        //cout << CN0_B << endl;
                        flag3++;
                        i += (k + 1);
                        k = 0;
                    }
                    // ComPVT
                    if ((rawData[i + k] != ',') && (flag3 == 6))
                    {
                        //cout << rawData[i+k] << endl;
                        aux[k] = rawData[i + k];
                    }
                    else if ((rawData[i + k] == ',') && (flag3 == 6))
                    {
                        ComPVT = atof(aux);
                        //cout << aux << endl;
                        ClearCharArray(aux, 5);
                        //cout << ComPVT << endl;
                        flag3++;
                        i += (k + 1);
                        k = 0;
                    }
                    // ComRTK
                    if ((rawData[i + k] != '*') && (flag3 == 7))
                    {
                        //cout << rawData[i+k] << endl;
                        aux[k] = rawData[i + k];
                    }
                    else if ((rawData[i + k] == '*') && (flag3 == 7))
                    {
                        ComRTK = atof(aux);
                        //cout << aux << endl;
                        ClearCharArray(aux, 5);
                        //cout << ComRTK << endl;
                        k = 1000;
                    }
                }
                if(Perc != 100)
                {
                    // VT100 scape codes
                    // http://www.climagic.org/mirrors/VT100_Escape_Codes.html
                    printf("\033[5A\r");
                    printf("\033[J\r");
                }
                // PVT_A & PVT_B
                _connStatus.pvt_a = PVT_A;
                _connStatus.pvt_b = PVT_B;
                // RTK_A & RTK_B
                _connStatus.rtk_a = RTK_A;
                _connStatus.rtk_b = RTK_B;
                // ComPVT & ComRTK
                _connStatus.com_pvt = ComPVT;
                _connStatus.com_rtk = ComRTK;
                // CN0_A & CNO_B  dBHz
                _connStatus.cn0_a = CN0_A;
                _connStatus.cn0_b = CN0_B;
                int percent_bar = Perc;
                if (percent_bar>100){
                    percent_bar = percent_bar/100;
                }
                if (flag_connecting==1) {
                    if (flag_startup_header_printed==false) {
                        printf("PVT: Number of satellites available for PVT solution\n");
                        printf("RTK: Number of satellites available for RTK solution\n");
                        printf("ComPVT: Common satellites for A and B in PVT\n");
                        printf("ComRTK: Common satellites for A and B in RTK\n");
                        printf("CNO: Highest CNO reported\n");
                        flag_startup_header_printed = true;
                    }
                    printf("\tStartup - %3d %\n", Perc); // Section 9.1.2
                    cout << "\tPVT_A: " << PVT_A << "\tRTK_A: " << RTK_A << endl;
                    cout << "\tPVT_B: " << PVT_B << "\tRTK_B: " << RTK_B << endl;
                    cout << "\tComPVT: " << ComPVT << "\tComRTK: " << ComRTK << endl;
                    cout << "\tCNO_A: " << CN0_A << "dBHz\tCNO_B: " << CN0_B << "dBHz" << endl;
                    if (PVT_A >=12 & PVT_B>=12 & CN0_A>=47 & CN0_B>=47 & ComPVT>=12 & ComRTK>=12){
                        printf("\tExcellent Conditions\n");
                    }else if (PVT_A >=8 & PVT_A<=11 & PVT_B>=8 & PVT_B<=11 & CN0_A>=40 & CN0_A<=46 & CN0_B>=40 & CN0_B<=46 & ComPVT>=9 & ComPVT<=11 & ComRTK>=9 & ComRTK<=11)
                    {
                        printf("\tFair Conditions\n");
                    }else if(PVT_A <=8 & PVT_B<=7 & CN0_A<40 & CN0_B<40 & ComPVT<9 & ComRTK<9 )
                    {
                        printf("\tPoor Conditions\n");
                    }else{
                        printf("\tVery Poor Conditions\n");
                    }
                    // According to sec 8.3.3 GNSS Compass Signal Health Status
                    // Progress bar
                    print_progress_bar(round(percent_bar));
                    //
                    _connStatus.startup_complete = false;
                }else{
                    _connStatus.startup_complete = true;
                }
            }
        }
        else
        {
            j = 0;
        }
    }
}


vec3d ECEF2ENU(vec3d posECEF, vec3d lla){
    // Convert ECEF to NED
    // https://www.mathworks.com/help/aeroblks/directioncosinematrixeceftoned.html
    double lat_r = lla[0]*M_PI/180.0;// Lat in rad
    double lon_r = lla[1]*M_PI/180.0;// Long in rad
    cout << "lat:" << lat_r << ", lon:" << lon_r << endl;
    mat3d DCM;
    DCM.e00 = -1.0*sin(lat_r)*cos(lon_r);
    DCM.e01 = -1.0*sin(lat_r)*sin(lon_r);
    DCM.e02 = cos(lat_r);
    DCM.e10 = -1.0*sin(lon_r);
    DCM.e11 = cos(lon_r);
    DCM.e12 = 0.0;
    DCM.e20 = -1.0*cos(lat_r)*cos(lon_r);
    DCM.e21 = cos(lat_r)*sin(lon_r);
    DCM.e22 = -1.0*sin(lat_r);
    vec3d pos_ENU;
    pos_ENU.x = DCM.e00*posECEF.x + DCM.e01*posECEF.y + DCM.e02*posECEF.z;
    pos_ENU.y = DCM.e10*posECEF.x + DCM.e11*posECEF.y + DCM.e12*posECEF.z;
    pos_ENU.z = DCM.e20*posECEF.x + DCM.e21*posECEF.y + DCM.e22*posECEF.z;
    cout << DCM << endl;
    cout << pos_ENU << endl;
    //return pos_ENU;
}


// Basic loop so we can initilize our covariance parameters above
boost::array<double, 9ul> setCov(XmlRpc::XmlRpcValue rpc){
    // Output covariance vector
    boost::array<double, 9ul> output = { 0.0 };

    // Convert the RPC message to array
    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < 9; i++){
    ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    output[i] = (double)rpc[i];
    }
    return output;
}


// Basic loop so we can initilize our baseline position configuration above
vec3f setPos(XmlRpc::XmlRpcValue rpc){
    // Output covariance vector
    vec3f output(0.0f, 0.0f, 0.0f);

    // Convert the RPC message to array
    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < 3; i++){
        ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output[i] = (double)rpc[i];
    }
    return output;
}



void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index){
    // By default, the orientation of IMU is NED (North East Down).
    vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
    sensor_msgs::MagneticField msgMag;
    sensor_msgs::Imu msgIMU;
    sensor_msgs::NavSatFix msgGPS;
    nav_msgs::Odometry msgOdom;
    sensor_msgs::Temperature msgTemp;
    sensor_msgs::FluidPressure msgPres;
    ros::Time ACQ_time = ros::Time::now();

    // Own MSG
    vectornav::vn_time _TimeSyncIn;
    vectornav::vn_time _TimeGps;
    vectornav::vn_time _TimeGpsPps;
    vectornav::ins_status _INSstatus;

    // Time
    if (cd.hasTimeGpsPps()){
        uint64_t TimeGPS_PPS;
        _TimeGpsPps.header.stamp = ACQ_time;
        _TimeGpsPps.time_value = cd.timeGpsPps();  //nano secs since the last GPS PPS trigger
        pubTimeGpsPps.publish(_TimeGpsPps);
    }
    if (cd.hasTimeSyncIn()){
        uint64_t TimeSyncIn;
        _TimeSyncIn.header.stamp = ACQ_time;
        _TimeSyncIn.time_value = cd.timeSyncIn();  //nano secs time since the last SyncIn trigger
        pubTimeSyncIn.publish(_TimeSyncIn);
    }
    if (cd.hasTimeGps()){
        uint64_t TimeGPS;
        _TimeGps.header.stamp = ACQ_time;
        _TimeGps.time_value = cd.timeGps(); // The absolute GPS time since start of GPS epoch 1980 expressed in nano seconds
        pubTimeGps.publish(_TimeGps);
    }
    // IMU
    msgIMU.header.stamp = ACQ_time;
    msgIMU.header.frame_id = frame_id;
    if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration()){
        vec4f q = cd.quaternion();
        vec3f ar = cd.angularRate();
        vec3f acel = cd.acceleration();

        if (cd.hasAttitudeUncertainty()){
            vec3f orientationStdDev = cd.attitudeUncertainty();
            msgIMU.orientation_covariance[0] = orientationStdDev[1] * orientationStdDev[1];
            msgIMU.orientation_covariance[4] = orientationStdDev[0] * orientationStdDev[0];
            msgIMU.orientation_covariance[8] = orientationStdDev[2] * orientationStdDev[2];
        }
        //Quaternion message comes in as a Yaw (z) pitch (y) Roll (x) format
        if (tf_ned_to_enu){
            // If we want the orientation to be based on the reference label on the imu
            tf2::Quaternion tf2_quat(q[0],q[1],q[2],q[3]);
            geometry_msgs::Quaternion quat_msg;

            if(frame_based_enu){
                // Create a rotation from NED -> ENU
                tf2::Quaternion q_rotate;
                q_rotate.setRPY (M_PI, 0.0, M_PI/2);
                // Apply the NED to ENU rotation such that the coordinate frame matches
                tf2_quat = q_rotate*tf2_quat;
                quat_msg = tf2::toMsg(tf2_quat);

                // Since everything is in the normal frame, no flipping required
                msgIMU.angular_velocity.x = ar[0];
                msgIMU.angular_velocity.y = ar[1];
                msgIMU.angular_velocity.z = ar[2];

                msgIMU.linear_acceleration.x = acel[0];
                msgIMU.linear_acceleration.y = acel[1];
                msgIMU.linear_acceleration.z = acel[2];
            }
            else{
                // put into ENU - swap X/Y, invert Z
                quat_msg.x = q[1];
                quat_msg.y = q[0];
                quat_msg.z = -q[2];
                quat_msg.w = q[3];

                // Flip x and y then invert z
                msgIMU.angular_velocity.x = ar[1];
                msgIMU.angular_velocity.y = ar[0];
                msgIMU.angular_velocity.z = -ar[2];
                // Flip x and y then invert z
                msgIMU.linear_acceleration.x = acel[1];
                msgIMU.linear_acceleration.y = acel[0];
                msgIMU.linear_acceleration.z = -acel[2];

                if (cd.hasAttitudeUncertainty()){
                    vec3f orientationStdDev = cd.attitudeUncertainty();
                    msgIMU.orientation_covariance[0] = orientationStdDev[1]*orientationStdDev[1]*M_PI/180; // Convert to radians pitch
                    msgIMU.orientation_covariance[4] = orientationStdDev[0]*orientationStdDev[0]*M_PI/180; // Convert to radians Roll
                    msgIMU.orientation_covariance[8] = orientationStdDev[2]*orientationStdDev[2]*M_PI/180; // Convert to radians Yaw
                }
            }
            msgIMU.orientation = quat_msg;
        }
        else{
            msgIMU.orientation.x = q[0];
            msgIMU.orientation.y = q[1];
            msgIMU.orientation.z = q[2];
            msgIMU.orientation.w = q[3];

            msgIMU.angular_velocity.x = ar[0];
            msgIMU.angular_velocity.y = ar[1];
            msgIMU.angular_velocity.z = ar[2];
            msgIMU.linear_acceleration.x = acel[0];
            msgIMU.linear_acceleration.y = acel[1];
            msgIMU.linear_acceleration.z = acel[2];
        }
        msgOdom.pose.pose.orientation.x = msgIMU.orientation.x;
        msgOdom.pose.pose.orientation.y = msgIMU.orientation.y;
        msgOdom.pose.pose.orientation.z = msgIMU.orientation.z;
        msgOdom.pose.pose.orientation.w = msgIMU.orientation.w;

        msgOdom.twist.twist.linear.x = msgIMU.angular_velocity.x;
        msgOdom.twist.twist.linear.y = msgIMU.angular_velocity.y;
        msgOdom.twist.twist.linear.z = msgIMU.angular_velocity.z;

        msgOdom.twist.twist.angular.x = msgIMU.linear_acceleration.x;
        msgOdom.twist.twist.angular.y = msgIMU.linear_acceleration.y;
        msgOdom.twist.twist.angular.z = msgIMU.linear_acceleration.z;
        // Covariances pulled from parameters
        msgIMU.angular_velocity_covariance = angular_vel_covariance;
        msgIMU.linear_acceleration_covariance = linear_accel_covariance;
        pubIMU.publish(msgIMU);
    }
    if (cd.hasYawPitchRoll()){
        vec3f ypr = cd.yawPitchRoll();
    }

    if (cd.hasPositionUncertaintyEstimated()){
        // The estimated uncertainty (1 Sigma) in the current position estimate, given in meters
        float INSposU;
        vectornav::ecef_uncert _INSPos_U;
        _INSPos_U.header.stamp = ACQ_time;
        _INSPos_U.ecef_pos_un = cd.positionUncertaintyEstimated();
        pubECEFun.publish(_INSPos_U);
    }

    // Magnetic Field
    if (cd.hasMagnetic()){
        vec3f mag = cd.magnetic();
        msgMag.header.stamp = ACQ_time;
        msgMag.header.frame_id = msgIMU.header.frame_id;
        msgMag.magnetic_field.x = mag[0];
        msgMag.magnetic_field.y = mag[1];
        msgMag.magnetic_field.z = mag[2];
        pubMag.publish(msgMag);
        //cout << "Binary Async MagneticField: " << mag << endl;
    }

    // GPS
    msgGPS.header.stamp = ACQ_time;
    msgGPS.header.frame_id = frame_id;
    if (cd.hasPositionEstimatedLla() && cd.hasPositionEstimatedEcef() && cd.hasInsStatus()){
        vec3d lla = cd.positionEstimatedLla();
        msgGPS.latitude = lla[0];
        msgGPS.longitude = lla[1];
        msgGPS.altitude = lla[2];
        pubGPS.publish(msgGPS);
        // cout << "Binary Async GPS_LLA: " << lla << endl;

        msgOdom.header.stamp = ACQ_time;
        msgOdom.header.frame_id = msgIMU.header.frame_id;
        vec3d pos = cd.positionEstimatedEcef();
        if (!flag_origin){
            pos_o = pos;
            flag_origin = 1;
        }
        if (ENU_flag){
            pos = ECEF2ENU(pos,lla);
        }
        msgOdom.pose.pose.position.x = pos[0];
        msgOdom.pose.pose.position.y = pos[1];
        msgOdom.pose.pose.position.z = pos[2];
        // cout << "Binary Async GPS_ECEF: " << pos << endl;

        pubOdom.publish(msgOdom);
        // INS STATUS
        uint16_t  INS_status;
        INS_status = cd.insStatus();
        /*
        Indicates the current mode of the INS filter.
        0 = Not tracking. GNSS Compass is initializing. Output heading is based on
        magnetometer measurements.
        1 = Aligning. INS Filter is dynamically aligning. For a stationary startup: GNSS Compass has initialized and INS Filter is
        aligning from the magnetic heading to the GNSS Compass heading. For a dynamic startup: INS Filter has initialized and is dynamically aligning
        to True North heading. In operation, if the INS Filter drops from INS Mode 2 back down to 1, the attitude uncertainty has increased above 2 degrees.
        2 = Tracking. The INS Filter is tracking and operating within specification.
        3 = Loss of GNSS. A GNSS outage has lasted more than 45 seconds. The INS Filter will no longer update the position and velocity outputs, but the
        attitude remains valid.
        */
        int INS_mode        = INS_status && 0b00000011;

        // Indicates whether the GNSS has a proper fix
        int INS_GNNSS_Fix   = INS_status && 0b00000100;
        INS_GNNSS_Fix = INS_GNNSS_Fix  >> 2;

        //Sensor measurement error code. See table below.
        //0 = No errors detected.
        // High if IMU communication error is detected.
        int INS_IMU_error   = INS_status && 0b00010000;
        INS_IMU_error = INS_IMU_error >> 4;
        // High if Magnetometer or Pressure sensor error is detected.
        int INS_MAG_error   = INS_status && 0b00100000;
        INS_MAG_error = INS_MAG_error >> 5;
        // High if GNSS communication error is detected
        int INS_GNSS_error  = INS_status && 0b01000000;
        INS_GNSS_error = INS_GNSS_error >> 6;

        // In stationary operation, if set the INS Filter has fully aligned to the GNSS
        //Compass solution.
        //In dynamic operation, the GNSS Compass solution is currently aiding the INS Filter heading solution.
        int INS_GNSS_Heading= INS_status && 0b0000000100000000;
        INS_GNSS_Heading = INS_GNSS_Heading >> 8;

        // Indicates if the GNSS compass is operational and reporting a heading solution.
        int INS_GNSS_Compass= INS_status && 0b0000001000000000;
        INS_GNSS_Compass = INS_GNSS_Compass >> 9;

        _INSstatus.header.stamp =     ACQ_time;
        _INSstatus.ins_mode =         INS_mode;
        _INSstatus.ins_gnss_fix =     INS_GNNSS_Fix;
        _INSstatus.ins_imu_error =    INS_IMU_error;
        _INSstatus.ins_mag_error =    INS_MAG_error;
        _INSstatus.ins_gnss_error =   INS_GNSS_error;
        _INSstatus.ins_gnss_heading = INS_GNSS_Heading;
        _INSstatus.ins_gnss_compass = INS_GNSS_Compass;
        pubINSstatus.publish(_INSstatus);
    }

    // Temperature
    if (cd.hasTemperature()){
        float temp = cd.temperature();
        msgTemp.header.stamp = ACQ_time;
        msgTemp.header.frame_id = msgIMU.header.frame_id;
        msgTemp.temperature = temp;
        pubTemp.publish(msgTemp);
    }

    // Barometer
    if (cd.hasPressure()){
        float pres = cd.pressure();
        msgPres.header.stamp = ACQ_time;
        msgPres.header.frame_id = msgIMU.header.frame_id;
        msgPres.fluid_pressure = pres;
        pubPres.publish(msgPres);
    }

}

// ------------------------------------------------------------------------------------ MAIN

int main(int argc, char *argv[]) {
    // ROS node init
    ros::init(argc, argv, "vectornav");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Publishers
    pubIMU     = n.advertise<sensor_msgs::Imu>("vectornav/IMU", 1000);
    pubMag     = n.advertise<sensor_msgs::MagneticField>("vectornav/Mag", 1000);
    pubGPS     = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS", 1000);
    pubOdom    = n.advertise<nav_msgs::Odometry>("vectornav/Odom", 1000);
    pubTemp    = n.advertise<sensor_msgs::Temperature>("vectornav/Temp", 1000);
    pubPres    = n.advertise<sensor_msgs::FluidPressure>("vectornav/Pres", 1000);
    pubGPSConStatus = n.advertise<vectornav::gps_conn_status>("vectornav/GPSConStatus", 1000);
    pubTimeSyncIn = n.advertise<vectornav::vn_time>("vectornav/TimeSyncIn", 1000);
    pubTimeGpsPps = n.advertise<vectornav::vn_time>("vectornav/TimeGpsPps", 1000);
    pubTimeGps    = n.advertise<vectornav::vn_time>("vectornav/pubTimeGps", 1000);
    pubINSstatus  = n.advertise<vectornav::ins_status>("vectornav/INSStatus", 1000);
    pubECEFun     = n.advertise<vectornav::ecef_uncert>("vectornav/INS_ECEF_Unc", 1000);

    // Serial Port Settings
    string SensorPort;
    int SensorBaudrate;
    int async_output_rate;

    // Sensor IMURATE (800Hz by default, used to configure device)
    int SensorImuRate;

    // Load all params
    // pn.param<type_of_data>(Param_name, Param_value, default_value)
    pn.param<std::string>("frame_id", frame_id, "vectornav");
    pn.param<bool>("tf_ned_to_enu", tf_ned_to_enu, false);
    pn.param<bool>("frame_based_enu", frame_based_enu, false);
    pn.param<bool>("ECEF2ENU", ENU_flag, false);
    pn.param<int>("async_output_rate", async_output_rate, 20);
    pn.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
    pn.param<int>("serial_baud", SensorBaudrate, 115200);
    pn.param<int>("fixed_imu_rate", SensorImuRate, 800);
    pn.param<bool>("wait_for_GNSS_startup", wait_for_GNSS_startup, false);

    //Call to set covariances
    if (pn.getParam("linear_accel_covariance", rpc_temp)) {
        linear_accel_covariance = setCov(rpc_temp);
    }
    if (pn.getParam("angular_vel_covariance", rpc_temp)) {
        angular_vel_covariance = setCov(rpc_temp);
    }
    if (pn.getParam("orientation_covariance", rpc_temp)) {
        orientation_covariance = setCov(rpc_temp);
    }
    //Call to set antenna A offset
    if (pn.getParam("Antenna_A_offset", rpc_temp)) {
        Antenna_A_offset = setPos(rpc_temp);
    }
    //Call to set baseline position configuration
    if (pn.getParam("baseline_position", rpc_temp)){
        baseline_position = setPos(rpc_temp);
    }

    ROS_INFO("Connecting to: %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);
    // This example walks through using the VectorNav C++ Library to connect to
    // and interact with a VectorNav sensor.

    // Now let's create a VnSensor object and use it to connect to our sensor.
    // vnprog/src/sensors.cpp
    VnSensor vs;

    // NEW block
    // Default baudrate variable
    int defaultBaudrate;
    // Run through all of the acceptable baud rates until we are connected
    // Looping in case someone has changed the default
    bool baudSet = false;
    while(!baudSet){
        // Make this variable only accessible in the while loop
        static int i = 0;
        defaultBaudrate = vs.supportedBaudrates()[i];
        ROS_INFO("Connecting with default at %d", defaultBaudrate);
        // Default response was too low and retransmit time was too long by default.
        // They would cause errors
        //vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
        //vs.setRetransmitDelayMs(50);  // Retransmit every 50 ms

        // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
        // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
        // All other values seem to work fine.
        try{
            // Connect to sensor at it's default rate
            if(defaultBaudrate != 128000 && SensorBaudrate != 128000)
            {
                vs.connect(SensorPort, defaultBaudrate);
                // Issues a change baudrate to the VectorNav sensor and then
                // reconnects the attached serial port at the new baudrate.
                vs.changeBaudRate(SensorBaudrate);
                // Only makes it here once we have the default correct
                ROS_INFO("Connected baud rate is %d",vs.baudrate());
                baudSet = true;
            }
        }
            // Catch all oddities
        catch(...){
            // Disconnect if we had the wrong default and we were connected
            vs.disconnect();
            ros::Duration(0.2).sleep();
        }
        // Increment the default iterator
        i++;
        // There are only 9 available data rates, if no connection
        // made yet possibly a hardware malfunction?
        if(i > 8)
        {
            break;
        }
    }
    // ................


    // Now we verify connection (Should be good if we made it this far)
    if (vs.verifySensorConnectivity()) {
        ROS_INFO("Device connection established");
    }else{
        ROS_ERROR("No device communication");
    }

    // Let's query the sensor's model number.
    string mn = vs.readModelNumber();
    ROS_INFO("Model Number: %s\n", mn.c_str());

    ROS_INFO("Restarting factory configuration .....................................");
    //vs.restoreFactorySettings();
    Thread::sleepSec(3);

    // Let's do some simple reconfiguration of the sensor. As it comes from the
    // factory, the sensor outputs asynchronous data at 40 Hz.
    ROS_INFO("New resgister configuration.........................................");
    vs.writeAsyncDataOutputFrequency(async_output_rate); // see table 6.2.8
    uint32_t newHz = vs.readAsyncDataOutputFrequency();

    //added by Harold. Redundant variable to ensure
    ImuRateConfigurationRegister IMUR = vs.readImuRateConfiguration();
    IMUR.imuRate = SensorImuRate;
    vs.writeImuRateConfiguration(IMUR);
    IMUR = vs.readImuRateConfiguration();

    /* VPE Resgister - section 9.3.1 */
    VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
    // Enable *********************************************************************
    vpeReg.enable = VPEENABLE_ENABLE;
    // Heading Mode ***************************************************************
    // Section 3.4.3 de VN100manual.pdf
    vpeReg.headingMode = HEADINGMODE_RELATIVE;
    // vpeReg.headingMode = HEADINGMODE_ABSOLUTE;
    // vpeReg.headingMode = HEADINGMODE_INDOOR;
    // Filtering Mode *************************************************************
    //vpeReg.filteringMode = VPEMODE_OFF;
    vpeReg.filteringMode = VPEMODE_MODE1;
    // Tuning Mode ****************************************************************
    // vpeReg.tuningMode = VPEMODE_OFF;
    vpeReg.tuningMode = VPEMODE_MODE1;
    vs.writeVpeBasicControl(vpeReg);

    /* INS Resgister Section 10.3.1 */
    InsBasicConfigurationRegisterVn300 InsReg = vs.readInsBasicConfigurationVn300();
    // Scenario *******************************************************************
    //InsReg.scenario = SCENARIO_AHRS;
    //InsReg.scenario = SCENARIO_INSWITHPRESSURE;
    //InsReg.scenario = SCENARIO_INSWITHOUTPRESSURE;
    InsReg.scenario = SCENARIO_GPSMOVINGBASELINEDYNAMIC; // GNSS moving baseline for dynamic applications.
    //InsReg.scenario = SCENARIO_GPSMOVINGBASELINESTATIC;
    // Ahrs Aiding ****************************************************************
    //AHRS aiding provides the ability to switch to using the magnetometer to stabilize heading during times when
    // the device is stationary and the GNSS compass is not available. AHRS aiding also helps to eliminate large
    // updates in the attitude solution during times when heading is weakly observable, such as at startup.
    InsReg.ahrsAiding = 1;
    // Estimation Base line *******************************************************
    // Enables GNSS compass baseline estimation by INS.
    InsReg.estBaseline = 1;
    vs.writeInsBasicConfigurationVn300(InsReg);

    /* HSI Calibration Section 11.1.1 Magnetometer Calibration Control */
    MagnetometerCalibrationControlRegister hsiReg = vs.readMagnetometerCalibrationControl();
    // HSI Mode *******************************************************************
    // Controls the mode of operation for the onboard real-time magnetometer hard/soft iron compensation algorithm.
    // RUN: Runs the real-time hard/soft iron calibration. The algorithm will continue using its existing solution.
    // The algorithm can be started and stopped at any time by switching between the HSI_OFF and HSI_RUN state.
    hsiReg.hsiMode = HSIMODE_RUN;
    //hsiReg.hsiMode = HSIMODE_OFF; // Real-time hard/soft iron calibration algorithm is turned off.
    // HSI Output *****************************************************************
    // Controls the type of measurements that are provided as outputs from the magnetometer sensor and also
    //subsequently used in the attitude filter.
    hsiReg.hsiOutput = HSIOUTPUT_USEONBOARD;  // Onboard HSI is applied to the magnetic measurements
    //hsiReg.hsiOutput = HSIOUTPUT_NOONBOARD; // Onboard HSI is not applied to the magnetic measurements.
    vs.writeMagnetometerCalibrationControl(hsiReg);

    /* BaseLine Configuration */
    /// BaseLine and Antenna A offset Configuration
    vs.writeGpsAntennaOffset(Antenna_A_offset); // from YAML parameters
    GpsCompassBaselineRegister baseli_config = vs.readGpsCompassBaseline();
    baseli_config.position = baseline_position;
    // Uncertainty calculation
    float uncertainty_offset = 0.01;
    baseli_config.uncertainty = {baseli_config.position[0]*0.05+uncertainty_offset, baseli_config.position[1]*0.05+uncertainty_offset, baseli_config.position[2]*0.05+uncertainty_offset}; //5%
    vs.writeGpsCompassBaseline(baseli_config);
    /* Save on flash memory all configurations */
    vs.writeSettings();

    /* ------------------------------------------------------------------------------ */
    /* ------- Read all configurations ---------------------------------------------- */
    /* ------------------------------------------------------------------------------ */
    ROS_INFO("Async output frequency:\t%d Hz", newHz);
    ROS_INFO("IMU Frequency:\t\t%d Hz", IMUR.imuRate);
    /* VPE Resgister */
    ROS_INFO("...VPE Resgister....................................................");
    vpeReg = vs.readVpeBasicControl();
    ROS_INFO("Enable:\t\t%d", vpeReg.enable);
    ROS_INFO("Heading Mode:\t%d", vpeReg.headingMode);
    ROS_INFO("Filtering Mode:\t%d", vpeReg.filteringMode);
    ROS_INFO("Tuning Mode:\t%d", vpeReg.tuningMode);
    /* INS Resgister */
    ROS_INFO("...INS Resgister....................................................");
    InsReg = vs.readInsBasicConfigurationVn300();
    ROS_INFO("Scenario:\t%d", InsReg.scenario);
    ROS_INFO("AHRS Aiding:\t%d", InsReg.ahrsAiding);
    ROS_INFO("Base Line:\t%d", InsReg.estBaseline);
    /* HIS Calibration */
    ROS_INFO("...HIS Calibration..................................................");
    hsiReg = vs.readMagnetometerCalibrationControl();
    ROS_INFO("Mode:\t%d", hsiReg.hsiMode);
    ROS_INFO("Output:\t%d\n", hsiReg.hsiOutput);
    /* BaseLine Configuration */
    ROS_INFO("BaseLine Configuration..............................................");
    vec3f Ant_offset = vs.readGpsAntennaOffset();
    ROS_INFO("Antena A offset:\t[%.2f, %.2f, %.2f]", Ant_offset[0], Ant_offset[1], Ant_offset[2]);
    baseli_config = vs.readGpsCompassBaseline();
    ROS_INFO("Position:\t\t[%.2f, %.2f, %.2f]", baseli_config.position[0], baseli_config.position[1], baseli_config.position[2]);
    ROS_INFO("Uncertainty:\t\t[%.4f, %.4f, %.4f]\n", baseli_config.uncertainty[0], baseli_config.uncertainty[1], baseli_config.uncertainty[2]);
    ROS_INFO("Convertion Configuration..............................................");
    if (ENU_flag) {
        ROS_INFO("XYZ output mode: ENU");
    }else{
        ROS_INFO("XYZ output mode: ECEF");
    }
    if (tf_ned_to_enu){
        ROS_INFO("Quaternion orientation mode: ENU");
    }else{
        ROS_INFO("Quaternion orientation mode: END");
    }
    if (frame_based_enu && tf_ned_to_enu){
        ROS_INFO("Rotation mode: Mathematically\n");
    }else{
        if (!frame_based_enu && tf_ned_to_enu) {
            ROS_INFO("Rotation mode: Manually\n");
        } else {
            ROS_INFO("Rotation mode: No apply\n");
        }
    }

    BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            SensorImuRate / async_output_rate,  // update rate [ms]
            COMMONGROUP_QUATERNION
                | COMMONGROUP_TIMEGPSPPS
                | COMMONGROUP_TIMESYNCIN
                | COMMONGROUP_ANGULARRATE
                | COMMONGROUP_POSITION
                | COMMONGROUP_ACCEL
                | COMMONGROUP_MAGPRES,
            TIMEGROUP_NONE,
            IMUGROUP_NONE,
            GPSGROUP_NONE,
            ATTITUDEGROUP_YPRU, //<-- returning yaw pitch roll uncertainties
            INSGROUP_INSSTATUS
                | INSGROUP_POSU
                | INSGROUP_POSLLA
                | INSGROUP_POSECEF
                | INSGROUP_VELBODY
                | INSGROUP_ACCELECEF,
            GPSGROUP_NONE);

    vs.registerRawDataReceivedHandler(NULL, ConnectionState);
    ROS_INFO("Initial GNSS startup calibration ................................................");

    if (wait_for_GNSS_startup == true) {
        while (flag_connecting && ros::ok()) {
            vs.send("$VNRRG,98"); //GNSS Compass Startup Status. Section 8.3.2
            vs.send("$VNRRG,86"); //GNSS Compass Signal Health Status. Section 8.3.3
            pubGPSConStatus.publish(_connStatus);
        }
    }else{
        flag_connecting=0;   // Flag connecting = 0 forces avoiud GNSS calibration
    }

    if(flag_connecting==1){
        ROS_INFO("\t Aborted connection");
    }else{
        vs.unregisterRawDataReceivedHandler();
        Thread::sleepSec(2);
        vs.writeBinaryOutput1(bor);
        vs.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncMessageReceived);
        ROS_INFO("bound..............................................................");
    }

    while (!flag_connecting && ros::ok())
    {
        pubGPSConStatus.publish(_connStatus);
    }

    vs.unregisterAsyncPacketReceivedHandler();
    vs.disconnect();
    cout << "\tBye-bye" << endl;
    return 0;
}