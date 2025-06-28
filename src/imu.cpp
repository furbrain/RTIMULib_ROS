////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "RTIMULib.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#define GRAVITY 9.81

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh;
    uint64_t now;

    std::string settings_dir;
    std::string imu_frame = "imu";
    std::string mag_frame;
    if (!nh.getParam("settings_dir", settings_dir)){
        if (const char* home = std::getenv("ROS_HOME")) {
            settings_dir = home;
        } else if (const char* home = std::getenv("HOME")) {
            settings_dir = home; 
            settings_dir += "/.ros";
        }
    }
    nh.getParam("imu_frame", imu_frame); 
    if (!nh.getParam("mag_frame", mag_frame)) {
        mag_frame = imu_frame;
    }
     
    RTIMUSettings *settings = new RTIMUSettings(settings_dir.c_str(), "RTIMULib");

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_raw", 5);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag_raw", 5);
    //  This is an opportunity to manually override any settings before the call IMUInit

    //  set up IMU

    imu->IMUInit();

    //  this is a convenient place to change fusion parameters

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    //  now just process data

    while (ros::ok()) {
        //  poll at the rate recommended by the IMU

        usleep(imu->IMUGetPollInterval() * 1000);

        while (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            auto imu_ts = imuData.timestamp;

            ros::Time ts(imu_ts / 1000000, imu_ts % 1000000);

            sensor_msgs::Imu imu_msg;
            imu_msg.header.frame_id = imu_frame;
            imu_msg.header.stamp = ts;
            imu_msg.linear_acceleration.x = imuData.accel.x() * GRAVITY;
            imu_msg.linear_acceleration.y = imuData.accel.y() * GRAVITY;
            imu_msg.linear_acceleration.z = imuData.accel.z() * GRAVITY;
            imu_msg.angular_velocity.x = imuData.gyro.x();
            imu_msg.angular_velocity.y = imuData.gyro.y();
            imu_msg.angular_velocity.z = imuData.gyro.z();
            imu_pub.publish(imu_msg);

            sensor_msgs::MagneticField mag_msg;
            mag_msg.header.frame_id = mag_frame;
            mag_msg.header.stamp = ts;
            mag_msg.magnetic_field.x = imuData.compass.x() * 1e-6;
            mag_msg.magnetic_field.y = imuData.compass.y() * 1e-6;
            mag_msg.magnetic_field.z = imuData.compass.z() * 1e-6;
            mag_pub.publish(mag_msg);
        }
    }
}

