#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include "Eigen/Eigen"
#include <tf/transform_broadcaster.h>

using namespace Eigen;
using namespace std;

class RTKConverter
{
private:
    ros::NodeHandle& nh;
    ros::Subscriber odom_sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster tf_pub;

    enum State{init, convert} state;

    double begin_time;
    int msg_cnt;
    double x_sum, y_sum, z_sum;

    Quaterniond q_g_r;
    Vector3d T_g_r;

public:
    RTKConverter(ros::NodeHandle& nh_): nh(nh_)
    {
        odom_sub = nh.subscribe("/mavros/global_position/local", 100, &RTKConverter::odom_cb, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/global_odom", 100);

        state = init;

        begin_time = ros::Time::now().toSec();
        msg_cnt = 0;
        x_sum = y_sum = z_sum = 0.0;

        ROS_INFO("waiting for 5s to calibrate position...");
    }

    static inline Quaterniond get_quat(const nav_msgs::Odometry::ConstPtr& msg)
    {
        return Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if(state == init){
            if(msg->header.stamp.toSec() - begin_time <= 5){
                x_sum += msg->pose.pose.position.x;
                y_sum += msg->pose.pose.position.y;
                z_sum += msg->pose.pose.position.z;
                ++msg_cnt;
            } else{
                const Quaterniond& q = get_quat(msg);
                const Vector3d& q_z = q.toRotationMatrix().block<3, 1>(0, 2);
                if(q_z.z() <= 0){
                    ROS_ERROR("drone init attitude not correct!");
                    return;
                }

                //q_g_r = (Quaterniond::FromTwoVectors(q_z, Vector3d::UnitZ()) * q).inverse();
                //T_g_r = -q_g_r.toRotationMatrix() * Vector3d(x_sum / msg_cnt, y_sum / msg_cnt, z_sum / msg_cnt);

                q_g_r = Quaterniond(1, 0, 0, 0);
                T_g_r = -Vector3d(x_sum / msg_cnt, y_sum / msg_cnt, z_sum / msg_cnt);

                state = convert;
                ROS_INFO("begin converting...");
            }
        } else{
            const Vector3d& p = q_g_r.toRotationMatrix() * Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) + T_g_r;
            Quaterniond q = q_g_r * get_quat(msg);
            if(q.w() < 0)
                q = Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
            const Vector3d& v = q_g_r.toRotationMatrix() * Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, -msg->twist.twist.linear.z);
            
            //const Vector3d& p = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            //const Quaterniond& q = get_quat(msg);
            //const Vector3d& v = Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, -msg->twist.twist.linear.z);

            nav_msgs::Odometry msg_pub = *msg;
            msg_pub.pose.pose.position.x = p.x();
            msg_pub.pose.pose.position.y = p.y();
            msg_pub.pose.pose.position.z = p.z();
            msg_pub.pose.pose.orientation.w = q.w();
            msg_pub.pose.pose.orientation.x = q.x();
            msg_pub.pose.pose.orientation.y = q.y();
            msg_pub.pose.pose.orientation.z = q.z();
            msg_pub.twist.twist.linear.x = v.x();
            msg_pub.twist.twist.linear.y = v.y();
            msg_pub.twist.twist.linear.z = v.z();
            odom_pub.publish(msg_pub);

            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = msg->header.stamp;
            tf_msg.header.frame_id = "world";
            tf_msg.child_frame_id = "body";
            tf_msg.transform.translation.x = p.x();
            tf_msg.transform.translation.y = p.y();
            tf_msg.transform.translation.z = p.z();
            tf_msg.transform.rotation.w = q.w();
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_pub.sendTransform(tf_msg);
        }
        
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_converter");
    ros::NodeHandle nh;
    RTKConverter rtk_converter(nh);
    ros::spin();
    return 0;
}