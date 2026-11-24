#include "input.h"
#include <cmath>

RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    is_hover_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    toggle_reboot = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    // This mapping reads channels 0..7 below.
    if (pMsg->channels.size() < 8)
    {
        ROS_ERROR_THROTTLE(2.0, "[px4ctrl] RC message has fewer than 8 channels");
        return;
    }
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    mode = ((double)msg.channels[4] - 1000.0) / 1000.0;
    gear = ((double)msg.channels[5] - 1000.0) / 1000.0;
    reboot_cmd = ((double)msg.channels[7] - 1000.0) / 1000.0;

    check_validity();

    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }
    if (!have_init_last_reboot_cmd)
    {
        have_init_last_reboot_cmd = true;
        last_reboot_cmd = reboot_cmd;
    }

    // 1
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    else
        enter_hover_mode = false;

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_hover_mode = true;
    else
        is_hover_mode = false;

    // 2
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
            enter_command_mode = true;
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;

        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }

    // 3
    if (!is_hover_mode && !is_command_mode)
    {
        if (last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
            toggle_reboot = true;
        else
            toggle_reboot = false;
    }
    else
        toggle_reboot = false;

    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
    }
}

bool RC_Data_t::check_centered()
{
    return abs(ch[0]) < 1e-5 &&
           abs(ch[1]) < 1e-5 &&
           abs(ch[2]) < 1e-5 &&
           abs(ch[3]) < 1e-5;
}

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    p.setZero();
    v.setZero();
    w.setZero();
    q.setIdentity();
    recv_new_msg = false;
    fault_latched = false;
    consecutive_good = 0;
    has_previous = false;
    previous_p.setZero();
    previous_q.setIdentity();
}

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    if (fault_latched)
        return;

    const ros::Time now = ros::Time::now();
    const ros::Time stamp = pMsg->header.stamp;
    const auto &pp = pMsg->pose.pose.position;
    const auto &qq = pMsg->pose.pose.orientation;
    const auto &vv = pMsg->twist.twist.linear;
    const auto &ww = pMsg->twist.twist.angular;

    Eigen::Vector3d next_p(pp.x, pp.y, pp.z);
    Eigen::Vector3d next_v(vv.x, vv.y, vv.z);
    Eigen::Vector3d next_w(ww.x, ww.y, ww.z);
    Eigen::Quaterniond next_q(qq.w, qq.x, qq.y, qq.z);

    const double age = (now - stamp).toSec();
    const double q_norm = next_q.norm();
    const char *fault = nullptr;
    if (stamp.isZero() || !std::isfinite(age) || age > 0.5 || age < -0.05)
        fault = "bad timestamp";
    else if (!next_p.allFinite() || !next_v.allFinite() || !next_w.allFinite()
             || !next_q.coeffs().allFinite() || std::abs(q_norm - 1.0) > 0.2)
        fault = "NaN/Inf or invalid quaternion";
    else if (next_v.norm() > 15.0 || next_p.norm() > 100.0)
        fault = "impossible speed or local position";
    else if (has_previous)
    {
        const double dt = (stamp - previous_stamp).toSec();
        if (dt <= 0.0)
            fault = "non-increasing timestamp";
        else if ((next_p - previous_p).norm() > 0.20 + 15.0 * dt)
            fault = "position jump";
        else if (previous_q.angularDistance(next_q) > 0.52 + 8.0 * dt)
            fault = "orientation jump";
    }

    if (fault)
    {
        ROS_ERROR("[px4ctrl] ODOM FAULT LATCHED: %s; autonomous control disabled", fault);
        fault_latched = true;
        consecutive_good = 0;
        return;
    }

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;
uav_utils::extract_odometry(pMsg, p, v, q, w);
#ifdef VEL_IN_BODY
    // Keep the existing body-frame velocity option.
    v = q.toRotationMatrix() * v;
#endif
    previous_stamp = stamp;
    previous_p = next_p;
    previous_q = next_q.normalized();
    has_previous = true;
    if (consecutive_good < 10)
        ++consecutive_good;
}

Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

State_Data_t::State_Data_t()
{
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{

    current_state = *pMsg;
}

ExtendedState_Data_t::ExtendedState_Data_t()
{
}

void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
{
    current_extended_state = *pMsg;
}

Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    double voltage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        voltage += pMsg->cell_voltage[i];
    }
    volt = 0.8 * volt + 0.2 * voltage; // Naive LPF, cell_voltage has a higher frequency

    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    percentage = pMsg->percentage;

    static ros::Time last_print_t = ros::Time(0);
    if (percentage > 0.05)
    {
        if ((rcv_stamp - last_print_t).toSec() > 10)
        {
            ROS_INFO("[px4ctrl] Voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
    else
    {
        if ((rcv_stamp - last_print_t).toSec() > 1)
        {
            // ROS_ERROR("[px4ctrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
}
