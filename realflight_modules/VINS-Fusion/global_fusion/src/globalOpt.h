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

#pragma once
#include <vector>
#include <list>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <atomic>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"

using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void optimize();

	// format t, tx,ty,tz,qw,qx,qy,qz
	vector<pair<double, vector<double>>> localPoseMap;
	vector<pair<double, vector<double>>> globalPoseMap;
	vector<pair<double, vector<double>>> GPSPositionMap;
	bool initGPS;
	atomic<bool> newGPS;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;

};

template<class IterType, class ValType, class GetValFunc>
IterType b_search(IterType begin, IterType end, ValType val, GetValFunc&& get_val)
{
    const IterType none = end;

    while(end - begin > 1){
        IterType mid = begin + (end - begin) / 2;
        const ValType mid_val = get_val(mid);
        if(mid_val == val)
            return mid;
        else if(val < mid_val)
            end = mid;
        else
            begin = mid;
    }

    if(get_val(begin) == val)
        return begin;
    else
        return none;
}