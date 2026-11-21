/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry;
double last_vio_t = -1;
double last_gps_t = -1;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    // by zusiliang: downsample to 10hz
    double t = GPS_msg->header.stamp.toSec();
    if(t - last_gps_t < 0.1)
        return;
    last_gps_t = t;

    gpsQueue.push(GPS_msg);
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);


    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            //int numSats = GPS_msg->status.service;
            double pos_accuracy = GPS_msg->position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            break;
        }
        else if(gps_t < t - 0.01)
            gpsQueue.pop();
        else if(gps_t > t + 0.01)
            break;
    }

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    
    // add by zusiliang
    odometry.twist = pose_msg->twist;

    pub_global_odometry.publish(odometry);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    ros::Subscriber sub_GPS = n.subscribe("/gps", 100, GPS_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    ros::spin();
    return 0;
}
