#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include "Eigen/Eigen"
#include <tf/transform_broadcaster.h>

using namespace Eigen;
using namespace std;

constexpr double cov_thr = 1e-3;
constexpr double rtk_delay = -0.05;

class RTKFusion
{
private:
    ros::NodeHandle& nh;
    ros::Subscriber rtk_sub;
    ros::Subscriber vio_sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster tf_pub;

    enum State{init, fuse} state;

    Quaterniond q_g_r;  // q from global to rtk (init)
    Vector3d T_g_r;  // t from global to rtk (init)

    Vector3d T_v_g;  // t from vio to global

    double last_rtk_t;

    double last_vio_t;
    Vector3d last_vio_pos;

public:
    RTKFusion(ros::NodeHandle& nh_): nh(nh_)
    {
        rtk_sub = nh.subscribe("/mavros/global_position/local", 100, &RTKFusion::rtk_cb, this);
        vio_sub = nh.subscribe("/vins_fusion/imu_propagate", 100, &RTKFusion::vio_cb, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/global_odom", 100);

        state = init;
        
        T_v_g.setZero();
        last_rtk_t = -1;
        last_vio_t = -1;

        ROS_INFO("waiting for 5s to calibrate position...");
    }

    void rtk_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        static double begin_time = ros::Time::now().toSec();
        static int msg_cnt = 0;
        static double x_sum = 0, y_sum = 0, z_sum = 0;

        if(state == init){
            if(msg->header.stamp.toSec() - begin_time <= 5){
                x_sum += msg->pose.pose.position.x;
                y_sum += msg->pose.pose.position.y;
                z_sum += msg->pose.pose.position.z;
                ++msg_cnt;
            } else{
                const Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
                const Vector3d& q_z = q.toRotationMatrix().block<3, 1>(0, 2);
                if(q_z.z() <= 0){
                    ROS_ERROR("drone init attitude not correct!");
                    return;
                }

                q_g_r = (Quaterniond::FromTwoVectors(q_z, Vector3d::UnitZ()) * q).inverse();
                T_g_r = -q_g_r.toRotationMatrix() * Vector3d(x_sum / msg_cnt, y_sum / msg_cnt, z_sum / msg_cnt);

                state = fuse;
                ROS_INFO("begin fusing...");
            }
        } else{
            if(msg->header.stamp.toSec() - last_rtk_t < 0.05)
                return;
            last_rtk_t = msg->header.stamp.toSec();
            if(last_vio_t < 0)
                return;
            if(msg->pose.covariance[0] > cov_thr){
                ROS_WARN("rtk cov: %.6f, this rtk msg is abandoned", msg->pose.covariance[0]);
                return;
            }

            const Vector3d& p = q_g_r.toRotationMatrix() * Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) + T_g_r;
            const Vector3d& v = q_g_r.toRotationMatrix() * Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, -msg->twist.twist.linear.z);
            const Vector3d& corr_vio_p = last_vio_pos + v * (msg->header.stamp.toSec() - last_vio_t + rtk_delay);
            T_v_g = p - corr_vio_p;

            cout << T_v_g.norm() << endl;
        }
        
    }

    void vio_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        last_vio_t = msg->header.stamp.toSec();
        last_vio_pos = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        const Vector3d p = last_vio_pos + T_v_g;

        nav_msgs::Odometry msg_pub = *msg;
        msg_pub.pose.pose.position.x = p.x();
        msg_pub.pose.pose.position.y = p.y();
        msg_pub.pose.pose.position.z = p.z();
        odom_pub.publish(msg_pub);

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "world";
        tf_msg.child_frame_id = "body";
        tf_msg.transform.translation.x = p.x();
        tf_msg.transform.translation.y = p.y();
        tf_msg.transform.translation.z = p.z();
        tf_msg.transform.rotation = msg->pose.pose.orientation;
        tf_pub.sendTransform(tf_msg);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_fusion");
    ros::NodeHandle nh;
    RTKFusion rtk_fusion(nh);
    ros::spin();
    return 0;
}