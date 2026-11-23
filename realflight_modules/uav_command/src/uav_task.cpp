#include <uav_command/uav_task.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"

namespace UAV_plan
{

    uav_command::uav_command()
    {
        land_flag = 0;
        landing = 0;
        task_flag = takeoff;
        Takeoff_land.takeoff_land_cmd = 1; // takeoff:1   land:2
        ego_setpoint.header.frame_id = "world";
        debug = 1;
        ready = false;
        receive_srvinfo = false;
        yaw = 0;
        // 圆环位置、方向初始化
        circle_static0_x = 0.0;
        circle_static0_y = 0.0;
        circle_static0_z = 0.0;
        circle_static0_direction_x = 0.0;
        circle_static0_direction_y = 0.0;
        circle_static0_direction_z = 0.0;

        circle_move0_x = 0.0;
        circle_move0_y = 0.0;
        circle_move0_z = 0.0;
        circle_move0_direction_x = 0.0;
        circle_move0_direction_y = 0.0;
        circle_move0_direction_z = 0.0;
        // init px4ctrl
        px4_setpoint.header.stamp = ros::Time::now();
        px4_setpoint.position.x = 0;
        px4_setpoint.position.y = 0;
        px4_setpoint.position.z = 2.5;

        px4_setpoint.velocity.x = 0;
        px4_setpoint.velocity.y = 0;
        px4_setpoint.velocity.z = 0;

        px4_setpoint.acceleration.x = 0;
        px4_setpoint.acceleration.y = 0;
        px4_setpoint.acceleration.z = 0;

        px4_setpoint.jerk.x = 0;
        px4_setpoint.jerk.y = 0;
        px4_setpoint.jerk.z = 0;

        px4_setpoint.yaw = 0;
        px4_setpoint.yaw_dot = 0;

        px4_flag = true;

        // init ego_planner
        ego_setpoint.pose.position.x = 0;
        ego_setpoint.pose.position.y = 0;
        ego_setpoint.pose.position.z = 2.5;
        theta = 0;
        ego_setpoint.pose.orientation.w = cos(theta / 2.0);
        ego_setpoint.pose.orientation.x = 0;
        ego_setpoint.pose.orientation.y = 0;
        ego_setpoint.pose.orientation.z = sin(theta / 2.0);

        ego_flag = true;
        msg_valid = false;
        next = false;
        // 数组索引值初始化
        index_in_vector = 0;

        land_delay = 0;
    }

    uav_command::~uav_command()
    {
    }

    void uav_command::init(ros::NodeHandle &nh)
    {
        // these parameters are in custom world frame
        //  x -- forward
        //  y -- right
        // y_sim --left
        //  original point of world frame is at the bottom left
        nh.param("/uav_command/task/origin_x", origin_x_, 0.65);
        nh.param("/uav_command/task/origin_y", origin_y_, 5.77);
        nh.param("/uav_command/task/right", right, 3.0);
        nh.param("/uav_command/task/left", left, 2.5);
        nh.param("/uav_command/task/cylinder_x_before", cylinder_before_[0], 24.0);
        nh.param("/uav_command/task/cylinder_y_before", cylinder_before_[1], 9.0);
        nh.param("/uav_command/task/cylinder_z_before", cylinder_before_[2], 1.5);
        nh.param("/uav_command/task/cylinder_yaw_before", cylinder_before_[3], 1.5);
        nh.param("/uav_command/task/chunnel_x_before", chunnel_before_[0], 15.0);
        nh.param("/uav_command/task/chunnel_y_before", chunnel_before_[1], 7.0);
        nh.param("/uav_command/task/chunnel_z_before", chunnel_before_[2], 1.5);
        nh.param("/uav_command/task/chunnel_yaw_before", chunnel_before_[3], 1.5);
        nh.param("/uav_command/task/chunnel_x_start", chunnel_start_[0], 15.0);
        nh.param("/uav_command/task/chunnel_y_start", chunnel_start_[1], 7.0);
        nh.param("/uav_command/task/chunnel_z_start", chunnel_start_[2], 1.5);
        nh.param("/uav_command/task/chunnel_yaw_start", chunnel_start_[3], 1.5);
        nh.param("/uav_command/task/chunnel_x_m", chunnel_m_[0], 15.0);
        nh.param("/uav_command/task/chunnel_y_m", chunnel_m_[1], 7.0);
        nh.param("/uav_command/task/chunnel_z_m", chunnel_m_[2], 1.5);
        nh.param("/uav_command/task/chunnel_yaw_m", chunnel_m_[3], 1.5);
        nh.param("/uav_command/task/chunnel_x_end", chunnel_end_[0], 15.0);
        nh.param("/uav_command/task/chunnel_y_end", chunnel_end_[1], 7.0);
        nh.param("/uav_command/task/chunnel_z_end", chunnel_end_[2], 1.5);
        nh.param("/uav_command/task/chunnel_yaw_end", chunnel_end_[3], 1.5);
        nh.param("/uav_command/task/maze_x_0", maze_0_[0], 24.0);
        nh.param("/uav_command/task/maze_y_0", maze_0_[1], 9.0);
        nh.param("/uav_command/task/maze_z_0", maze_0_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_0", maze_0_[3], 1.5);
        nh.param("/uav_command/task/maze_x_1", maze_1_[0], 24.0);
        nh.param("/uav_command/task/maze_y_1", maze_1_[1], 9.0);
        nh.param("/uav_command/task/maze_z_1", maze_1_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_1", maze_1_[3], 1.5);
        nh.param("/uav_command/task/maze_x_2", maze_2_[0], 24.0);
        nh.param("/uav_command/task/maze_y_2", maze_2_[1], 9.0);
        nh.param("/uav_command/task/maze_z_2", maze_2_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_2", maze_3_[3], 1.5);
        nh.param("/uav_command/task/maze_x_3", maze_3_[0], 24.0);
        nh.param("/uav_command/task/maze_y_3", maze_3_[1], 9.0);
        nh.param("/uav_command/task/maze_z_3", maze_3_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_3", maze_3_[3], 1.5);
        nh.param("/uav_command/task/maze_x_4", maze_4_[0], 24.0);
        nh.param("/uav_command/task/maze_y_4", maze_4_[1], 9.0);
        nh.param("/uav_command/task/maze_z_4", maze_4_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_4", maze_4_[3], 1.5);
        nh.param("/uav_command/task/maze_x_behind", maze_behind_[0], 24.0);
        nh.param("/uav_command/task/maze_y_behind", maze_behind_[1], 9.0);
        nh.param("/uav_command/task/maze_z_behind", maze_behind_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_behind", maze_behind_[3], 1.5);
        nh.param("/uav_command/task/circle_x_before", circle_before_[0], 24.0);
        nh.param("/uav_command/task/circle_y_before", circle_before_[1], 9.0);
        nh.param("/uav_command/task/circle_z_before", circle_before_[2], 1.5);
        nh.param("/uav_command/task/circle_yaw_before", circle_before_[3], 1.5);
        nh.param("/uav_command/task/circle_x_next", circle_next_[0], 24.0);
        nh.param("/uav_command/task/circle_y_next", circle_next_[1], 9.0);
        nh.param("/uav_command/task/circle_z_next", circle_next_[2], 1.5);
        nh.param("/uav_command/task/circle_yaw_next", circle_next_[3], 1.5);
        nh.param("/uav_command/task/land_x", land_[0], 1.0);
        nh.param("/uav_command/task/land_y", land_[1], 26.0);
        nh.param("/uav_command/task/land_z", land_[2], 1.0);
        nh.param("/uav_command/task/land_yaw", land_[3], 26.0);
        nh.param("/uav_command/task/cylinder_z_0", cylinder_z_0_, 1.5);
        nh.param("/uav_command/task/point_x_1", point1[0], 1.5);
        nh.param("/uav_command/task/point_y_1", point1[1], 1.5);
        nh.param("/uav_command/task/point_x_2", point2[0], 1.5);
        nh.param("/uav_command/task/point_y_2", point2[1], 1.5);
        nh.param("/uav_command/task/point_x_3", point3[0], 1.5);
        nh.param("/uav_command/task/point_y_3", point3[1], 1.5);
        nh.param("/uav_command/task/point_x_4", point4[0], 1.5);
        nh.param("/uav_command/task/point_y_4", point4[1], 1.5);
        is_vision_server_call = true;
        cylinder_index = 0;
        wader_point_index = 0;
    }

    // Does ego_planner arrive its target?
    bool uav_command::arrive1(double dst)
    {
        distance1 = pow(rc_odom.odom.pose.pose.position.x - ego_setpoint.pose.position.x, 2) + pow(rc_odom.odom.pose.pose.position.y - ego_setpoint.pose.position.y, 2);
        distance1 = sqrt(distance1);

        ROS_INFO("distance1: %.2f", distance1);

        if (distance1 < dst)
            return true;
        else
            return false;
    }

    // Does px4ctrl  arrive its target?
    bool uav_command::arrive2(double dst)
    {

        distance2 = pow(rc_odom.odom.pose.pose.position.x - px4_setpoint.position.x, 2) + pow(rc_odom.odom.pose.pose.position.y - px4_setpoint.position.y, 2);

        angle = abs(px4_setpoint.yaw - rc_odom.yaw);

        distance2 = sqrt(distance2);

        // ROS_INFO("distance2: %.2f", distance2);
        ROS_INFO("odom_yaw: %.2f", rc_odom.yaw);
        ROS_INFO("yaw_setpoint:%.2f", px4_setpoint.yaw);
        ROS_INFO("angle: %.2f", angle);

        if (px4_setpoint.yaw < 3.1 && px4_setpoint.yaw > -3.1)
        {
            // ROS_INFO("0 0 0");
            ROS_INFO("distance2: %.2f", distance2);
            if (distance2 < dst && angle < 0.2)
                return true;
            else
                return false;
        }

        if (3.1 < px4_setpoint.yaw || px4_setpoint.yaw < -3.1) // 0 --- M_PI  0 ---  -M_PI
        {
            // ROS_INFO("M_PI");
            if ((distance2 < dst && angle < 0.2) || (distance2 < dst && angle > 6.18))
                return true;
            else
                return false;
        }
    }
    // 启用前
    //  ego_flag = true;
    //  px4_flag = true;
    //  next = false;

    void uav_command::go2target(double x, double y, double z, double yaw)
    {
        ego_setpoint.header.stamp = ros::Time::now();
        ego_setpoint.pose.position.x = x;
        ego_setpoint.pose.position.y = y;
        ego_setpoint.pose.position.z = z;
        ego_setpoint.pose.orientation.w = cos(yaw / 2.0);
        ego_setpoint.pose.orientation.x = 0;
        ego_setpoint.pose.orientation.y = 0;
        ego_setpoint.pose.orientation.z = sin(yaw / 2.0);

        px4_setpoint.header.stamp = ros::Time::now();
        px4_setpoint.position.x = x;
        px4_setpoint.position.y = y;
        px4_setpoint.position.z = z;
        px4_setpoint.yaw = yaw;

        // 先用ego_planner 引导到目标位置
        // 再调用px4ctrl旋转到合适的角度
        // ego
        if (ego_flag)
        {
            ego_pub.publish(ego_setpoint);
        }
        ego_flag = false;
        if (arrive1(0.5) && next == false)
        {
            ROS_INFO("ego arrives at target!!!");
            px4_flag = false;
        }

        // px4
        if (!px4_flag)
        {
            px4_pub.publish(px4_setpoint);
        }
        if (arrive2(0.3) && next == false)
        {
            next = true;
            px4_flag = true;
            ros::Duration(1).sleep();
            ROS_INFO("px4 arrives at target!!!");
        }
    }

    // 启用前
    //  ego_flag = ture;
    //  next = false;

    void uav_command::ego(double x, double y, double z, double yaw, double dist)
    {
        ego_setpoint.header.stamp = ros::Time::now();
        ego_setpoint.pose.position.x = x;
        ego_setpoint.pose.position.y = y;
        ego_setpoint.pose.position.z = z;
        ego_setpoint.pose.orientation.w = cos(yaw / 2.0);
        ego_setpoint.pose.orientation.x = 0;
        ego_setpoint.pose.orientation.y = 0;
        ego_setpoint.pose.orientation.z = sin(yaw / 2.0);

        ROS_INFO("EGO_FLAG: %d", ego_flag);
        if (ego_flag)
        {
            ego_pub.publish(ego_setpoint);
        }
        ego_flag = false;

        if (arrive1(dist) && next == false)
        {
            ROS_INFO("ego arrives at target!!!");
            next = true;
        }
    }

    // 启用px4_flag = true;
    //  next = false;
    void uav_command::px4(double x, double y, double z, double yaw, double dist)
    {
        px4_setpoint.header.stamp = ros::Time::now();
        px4_setpoint.position.x = x;
        px4_setpoint.position.y = y;
        px4_setpoint.position.z = z;
        px4_setpoint.yaw = yaw;

        if (px4_flag)
        {
            px4_pub.publish(px4_setpoint);
        }

        if (arrive2(dist) && next == false)
        {
            next = true;
            px4_flag = false;
            ROS_INFO("px4 arrives at target!!!");
        }
    }

    void uav_command::land_planner()
    {

        rc_tag2world.waitfor_tag2world();

        rc_tag2world.loop = 1;

        if (rc_tag2world.detection == 1) // start langing process
        {
            if (!land_flag)
            {
                x = rc_tag2world.tag2world.pose.position.x;
                y = rc_tag2world.tag2world.pose.position.y;
                z = 1.5;
                yaw = rc_odom.yaw;

                px4(x, y, z, yaw, 0.1);

                if (debug)
                {
                    ROS_INFO("flying to land_tag: x:%0.1f  y:%0.1f  z:%0.1f", x, y, z);
                }

                debug = 0;

                if (next == true)
                {
                    land_flag = 1;
                    ROS_ERROR("lang_flag = 1");
                }
            }

            if (land_flag == 1 && !landing)
            {
                Takeoff_land.takeoff_land_cmd = 2;
                ROS_INFO("landing!!!");

                ros::Duration(1).sleep();
                TakeoffLand_pub.publish(Takeoff_land);
                landing = 1;

                ROS_ERROR("landing!!!");
            }
        }
    }

    void uav_command::task_planner()
    { // task_flag
        //  1 -- takeoff
        //  2 -- pass through cylinder0
        //  3 -- pass through cylinder1
        //  4 -- pass through cylinder2
        // 5  -- pass through cylinder3
        //  6 -- pass through  chunnel
        // 7 --pass through maze
        //  8 -- circle_static_0
        //  9 -- circle_move_0
        //  10 -- land

        // take_off

        if (task_flag == takeoff)
        {
            Takeoff_land.takeoff_land_cmd = 1;
            // TakeoffLand_pub.publish(Takeoff_land);

            // if(abs(rc_odom.odom.pose.pose.position.z - 1.0) < 0.1)
            // {
            task_flag = cylinder_before; //  circle_static_0 maze_behind maze_behind
            ROS_INFO("takeoff!!!");
            ego_flag = true;
            px4_flag = true;
            next = false;
            // }
        }
        //
        else if (task_flag == cylinder_before)
        {
            x = cylinder_before_[0]; // 6.00
            y = cylinder_before_[1]; // 0.00
            z = cylinder_before_[2];
            yaw = cylinder_before_[3];
        
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;
                task_flag = wader;
                next = false;
            }
        }
        
        // 蛇形转弯过圆柱
        // 第一个圆柱点为侧移，使用px4直接给点，后续都使用ego进行轨迹规划
        else if (task_flag == wader)
        {
            if (wader_point_index == 0)
            {
                ego(point1[0], point1[1], cylinder_z_0_, -0.118, 0.6);
                ROS_INFO("POINT 1 : %f %f", point1[0], point1[1]);
            }
            else if (wader_point_index == 1)
            {
                ego(point2[0], point2[1], cylinder_z_0_, 0.323, 0.6);
                ROS_INFO("POINT 2 : %f %f", point2[0], point2[1]);
            }
            else if (wader_point_index == 2)
            {
                ego(point3[0], point3[1], cylinder_z_0_, -0.416, 0.6);
                ROS_INFO("POINT 3 : %f %f", point3[0], point3[1]);
            }
            else if (wader_point_index == 3)
            {
                ego(point4[0], point4[1], cylinder_z_0_, 0.218, 0.6);
                ROS_INFO("POINT 4 : %f %f", point4[0], point4[1]);
            }
            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;
                wader_point_index = wader_point_index + 1;
                if (wader_point_index == 4)
                {
                    task_flag = chunnel_before;
                }
                else
                {
                    task_flag = wader;
                }
                next = false;
            }

            // 前三个柱子的下一任务都是cylinder,最后一个柱子的任务是隧道
        }
        // chunnel
        else if (task_flag == chunnel_before)
        {
            // ROS_INFO("Task Flag = %d ", task_flag);
            x = chunnel_before_[0]; // 16.06
            y = chunnel_before_[1]; //-3.51
            z = chunnel_before_[2];
            yaw = chunnel_before_[3];

            if (ego_flag == true)
            {
                ROS_INFO("chunnel_before !!! x:%.2f y:%.2f z:%.2f angle:%.2f", x, y, z, yaw);
            }
            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = chunnel_start;
                next = false;
            }
        }
        else if (task_flag == chunnel_start)
        {
            // ROS_INFO("Task Flag = %d ", task_flag);
            x = chunnel_start_[0]; // 16.06
            y = chunnel_start_[1]; //-3.51
            z = chunnel_start_[2];
            yaw = chunnel_start_[3];

            if (ego_flag == true)
            {
                ROS_INFO("chunnel_start !!! x:%.2f y:%.2f z:%.2f angle:%.2f", x, y, z, yaw);
            }
            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.5);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = chunnel_m;
                next = false;
            }
        }
        else if (task_flag == chunnel_m)
        {
            x = chunnel_m_[0];
            y = chunnel_m_[1];
            z = chunnel_m_[2];
            yaw = chunnel_m_[3];
            if (ego_flag == true)
            {
                ROS_INFO("chunnel_m !!! x:%.2f y:%.2f z:%.2f", x, y, z);
            }
            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);
            // ego(x, y, z, yaw, 0.35);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = chunnel_end; // maze_0
                next = false;
            }
        }
        else if (task_flag == chunnel_end)
        {
            x = chunnel_end_[0];
            y = chunnel_end_[1];
            z = chunnel_end_[2];
            yaw = chunnel_end_[3];
            if (ego_flag == true)
            {
                ROS_INFO("chunnel_end !!! x:%.2f y:%.2f z:%.2f", x, y, z);
            }
            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);
            // ego(x, y, z, yaw, 0.35);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_0; // maze_0
                next = false;
            }
        }
        else if (task_flag == maze_before)
        {
            x = maze_before_[0];
            y = maze_before_[1];
            z = maze_before_[2];
            yaw = maze_before_[3];

            if (ego_flag == true)
            {
                ROS_INFO("go to the point brfore gate0!!! x:%.2f y:%.2f z:%.2f", x, y, z);
            }
            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_0;
                next = false;
            }
        }
        else if (task_flag == maze_0)
        {
            x = maze_0_[0];
            y = maze_0_[1];
            z = maze_0_[2];
            yaw = maze_0_[3];

            if (ego_flag == true)
            {
                ROS_INFO("go to the point brfore gate0!!! x:%.2f y:%.2f z:%.2f", x, y, z);
            }
            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_1;
                next = false;
            }
        }
        // maze_0
        else if (task_flag == maze_1)
        {
            x = maze_1_[0];
            y = maze_1_[1];
            z = maze_1_[2];
            yaw = maze_1_[3];

            if (ego_flag == true)
                ROS_INFO("go through gate0 !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_behind;
                next = false;
            }
        }
        else if (task_flag == maze_2)
        {
            x = maze_2_[0];
            y = maze_2_[1];
            z = maze_2_[2];
            yaw = maze_2_[3];

            if (ego_flag == true)
                ROS_INFO("go to the point brfore gate1 !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_3;
                next = false;
            }
        }
        else if (task_flag == maze_3)
        {
            x = maze_3_[0];
            y = maze_3_[1];
            z = maze_3_[2];
            yaw = maze_3_[3];

            if (ego_flag == true)
                ROS_INFO("after go through gate1 !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_4;
                next = false;
            }
        }
        else if (task_flag == maze_4)
        {
            x = maze_4_[0];
            y = maze_4_[1];
            z = maze_4_[2];
            yaw = maze_4_[3];

            if (ego_flag == true)
                ROS_INFO("go to the point brfore gate2 !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw)
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_behind;
                next = false;
            }
        }
        else if (task_flag == maze_behind)
        {
            x = maze_behind_[0];
            y = maze_behind_[1];
            z = maze_behind_[2];
            yaw = maze_behind_[3];

            if (ego_flag == true)
            ROS_INFO("after go to circle !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = circle_before;
                next = false;
            }
        }
        else if (task_flag == circle_before)
        {
            x = circle_before_[0];
            y = circle_before_[1];
            z = circle_before_[2]; // 圆环圆心离地高度2-2.5m
            yaw = circle_before_[3];

            if (ego_flag == true)
                ROS_INFO("go to position that can see the circle!!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.4);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = circle_static_add;
                next = false;
            }
        }
        // circle_static前面的点
        else if (task_flag == circle_static_add)
        {
            srv_circle_info.request.obstacle_type = circle_static_0;
            srv_switch_mode_info.request.switch_to = 2;

            switch_modeinfo = vision_switch_mode_client.call(srv_switch_mode_info);
            ros::Duration(2.0).sleep();
            // if (is_vision_server_call)
            // {
            receive_srvinfo = vision_circle_client.call(srv_circle_info);
            // is_vision_server_call = false;
            //}

            // 切换视觉检测模式
            if (switch_modeinfo)
            {
                ROS_INFO("change mode to 2");
            }
            else
            {
                ROS_INFO("change mode error!!!");
            }
            ROS_INFO("circle type:%d", circle_static_0);
            if (receive_srvinfo)
            {
                ROS_INFO("get vision msg about static circle now.");
                receive_srvinfo = false;
                ego_flag = true;
                px4_flag = true;
                // msg_valid = srv_circle_info.response.msg_valid;
                // if (msg_valid)
                // {
                circle_static0_x = srv_circle_info.response.pos_x;
                circle_static0_y = srv_circle_info.response.pos_y;
                circle_static0_z = srv_circle_info.response.pos_z;
                circle_static0_direction_x = srv_circle_info.response.direction_x;
                circle_static0_direction_y = srv_circle_info.response.direction_y;
                circle_static0_direction_z = srv_circle_info.response.direction_z;

                ROS_INFO("get  vision_msg about ciecle_0 position!!! x:%.2f y:%.2f z:%.2f", circle_static0_x, circle_static0_y, circle_static0_z);
                // ROS_INFO("get  vision_msg about ciecle_0 orientation!!! dir_x:%.2f dir_y:%.2f dir_z:%.2f", circle_static0_direction_x, circle_static0_direction_y, circle_static0_direction_z);

                double dir_tmp = sqrt(circle_static0_direction_x * circle_static0_direction_x + circle_static0_direction_y * circle_static0_direction_y);

                x = circle_static0_x + 1.0 * circle_static0_direction_x / dir_tmp;
                y = circle_static0_y + 1.0 * circle_static0_direction_y / dir_tmp;
                z = circle_static0_z;
                yaw = M_PI;
                if (ego_flag == true)
                {
                    ROS_INFO("circle_static !!! x:%.2f y:%.2f z:%.2f", circle_static0_x, circle_static0_y, circle_static0_z);
                    ROS_INFO("move to circle_static_add !!! x:%.2f y:%.2f z:%.2f", x, y, z);
                }
                ego(x, y, z, yaw, 0.6);
                // px4(x, y, z, yaw, 0.2);

                if (next == true)
                {
                    next = false;
                    ego_flag = true;
                    px4_flag = true;
                    task_flag = circle_static_center;
                    next = false;
                }
            }
        }
        else if (task_flag == circle_static_center)
        {
            x = circle_static0_x;
            y = circle_static0_y;
            z = circle_static0_z;
            yaw = M_PI;
            if (ego_flag == true)
            {
                ROS_INFO("circle_static !!! x:%.2f y:%.2f z:%.2f", circle_static0_x, circle_static0_y, circle_static0_z);
                ROS_INFO("move to circle_static_center !!! x:%.2f y:%.2f z:%.2f", x, y, z);
            }
            ego(x, y, z, yaw, 0.6);
            // px4(x, y, z, yaw, 0.2);

            if (next == true)
            {
                next = false;
                ego_flag = true;
                px4_flag = true;
                task_flag = circle_static_0; 
                next = false;
            }
        }
        // circle_static后面的点
        else if (task_flag == circle_static_0)
        {

            double dir_tmp = sqrt(circle_static0_direction_x * circle_static0_direction_x + circle_static0_direction_y * circle_static0_direction_y);

            x = circle_static0_x - 1.0 * circle_static0_direction_x / dir_tmp;
            y = circle_static0_y - 1.0 * circle_static0_direction_y / dir_tmp;
            z = circle_static0_z;
            yaw = M_PI;
            if (ego_flag == true)
            {
                ROS_INFO("circle_static !!! x:%.2f y:%.2f z:%.2f", circle_static0_x, circle_static0_y, circle_static0_z);
                ROS_INFO("move to circle_static_point_behind !!! x:%.2f y:%.2f z:%.2f", x, y, z);
            }
            ego(x, y, z, yaw, 0.6);
            // px4(x, y, z, yaw, 0.2);

            if (next == true)
            {
                next = false;
                ego_flag = true;
                px4_flag = true;
                task_flag = circle_next; // 下一状态为观测动态圆环 circle_next
                next = false;
            }
        }
        else if (task_flag == circle_next)
        {
            x = circle_next_[0];
            y = circle_next_[1];
            z = circle_next_[2]; // 圆环圆心离地高度2-2.5m
            yaw = circle_next_[3];

            if (ego_flag == true)
                ROS_INFO("go to position that can see the circle2!!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;
                is_vision_server_call = true;
                task_flag = circle_move_add;
                next = false;
                is_vision_server_call = true;
            }
        }
        else if (task_flag == circle_move_add)
        {
            srv_circle_info.request.obstacle_type = circle_move_0;
            while (receive_srvinfo == false)
            { // 当没有接收到视觉消息一直循环
                receive_srvinfo = vision_circle_client.call(srv_circle_info);
                ROS_INFO("waiting for vision msg about move circle...");
            }
            ROS_INFO("get msg about circle move now.");
            if (receive_srvinfo)
            {
                ego_flag = true;
                px4_flag = true;
                ROS_INFO("get vision msg about circle move now.");
                circle_move0_x = srv_circle_info.response.pos_x;
                circle_move0_y = srv_circle_info.response.pos_y;
                circle_move0_z = srv_circle_info.response.pos_z;
                circle_move0_direction_x = srv_circle_info.response.direction_x;
                circle_move0_direction_y = srv_circle_info.response.direction_y;
                circle_move0_direction_z = srv_circle_info.response.direction_z;

                ROS_INFO("get  vision_msg about ciecle_0 position!!! x:%.2f y:%.2f z:%.2f", circle_move0_x, circle_move0_y, circle_move0_z);
                ROS_INFO("get  vision_msg about ciecle_0 orientation!!! dir_x:%.2f dir_y:%.2f dir_z:%.2f", circle_move0_direction_x, circle_move0_direction_y, circle_move0_direction_z);
                // 先飞到圆环正前方
                double dir_tmp = sqrt(circle_move0_direction_x * circle_move0_direction_x + circle_move0_direction_y * circle_move0_direction_y);

                x = circle_move0_x + 1.0 * circle_move0_direction_x / dir_tmp;
                y = circle_move0_y + 1.0 * circle_move0_direction_y / dir_tmp;
                z = circle_move0_z;
                // 待计算
                yaw = M_PI;

                if (ego_flag == true)
                {
                    ROS_INFO("circle_move !!! x:%.2f y:%.2f z:%.2f", circle_move0_x, circle_move0_y, circle_move0_z);
                    ROS_INFO("move to circle_move_add !!! x:%.2f y:%.2f z:%.2f", x, y, z);
                }
                ego(x, y, z, yaw, 0.6);
                if (next == true)
                {
                    next = false;
                    ego_flag = true;
                    px4_flag = true;
                    task_flag = circle_move_0;
                }
            }
        }
        // else if (task_flag == circle_move_center)
        // {
        //     x = circle_move0_x;
        //     y = circle_move0_y;
        //     z = circle_static0_z;
        //     yaw = M_PI;
        //     if (ego_flag == true)
        //     {
        //         ROS_INFO("circle_static !!! x:%.2f y:%.2f z:%.2f", circle_move0_x, circle_move0_y, circle_move0_z);
        //         ROS_INFO("move to circle_move_center !!! x:%.2f y:%.2f z:%.2f", x, y, z);
        //     }
        //     ego(x, y, z, yaw, 0.6);
        //     // px4(x, y, z, yaw, 0.2);

        //     if (next == true)
        //     {
        //         next = false;
        //         ego_flag = true;
        //         px4_flag = true;
        //         task_flag = circle_move_0; //
        //         next = false;
        //     }
        // }
        // circle_move
        else if (task_flag == circle_move_0)
        {
                
                
                double dir_tmp = sqrt(circle_move0_direction_x * circle_move0_direction_x + circle_move0_direction_y * circle_move0_direction_y);

                x = circle_move0_x - 1.5 * circle_move0_direction_x / dir_tmp;
                y = circle_move0_y - 1.5 * circle_move0_direction_y / dir_tmp;
                z = circle_move0_z;
                // 待计算
                yaw = M_PI;

                if (ego_flag == true)
                {
                    ROS_INFO("circle_move !!! x:%.2f y:%.2f z:%.2f", circle_move0_x, circle_move0_y, circle_move0_z);
                    ROS_INFO("move to circle_move_ !!! x:%.2f y:%.2f z:%.2f", x, y, z);
                }
                ego(x, y, z, yaw, 0.6);
                if (next == true)
                {
                    next = false;
                    ego_flag = true;
                    px4_flag = true;
                    task_flag = land; // 下一状态为飞向降落区
                }
            
        }
        // landing
        else if (task_flag == land)
        {
            x = land_[0];
            y = land_[1];
            z = land_[2];
            yaw = land_[3];

            if (ego_flag == true)
            {
                ROS_INFO("go to apriltag !!! x:%.2f y:%.2f z:%.2f", x, y, z);
            }
            ego(x, y, z, yaw, 0.5);

            if (next == true)
            {
                land_flag = 1;
                ROS_INFO("land flag=1.");
            }
            if (land_flag == 1)
            {
                Takeoff_land.takeoff_land_cmd = 2;
                land_delay++;
                if (land_delay > 30)
                {
                    ROS_ERROR("landing!!!");
                    // ros::Duration(1).sleep();
                    TakeoffLand_pub.publish(Takeoff_land);
                    // landing = 1;
                }
                ROS_ERROR("landing!!!");
            }
        }
        // publish landing cmd
        // else if (task_flag == land_end)
        // {
        //     land_flag = 1;
        //     ROS_INFO("land_flag = 1");
        //     Takeoff_land.takeoff_land_cmd = 2;
        //     land_delay++;
        //     if (land_delay > 100)
        //     {
        //         ROS_INFO("landing!!!");
        //         // ros::Duration(1).sleep();
        //         TakeoffLand_pub.publish(Takeoff_land);
        //         // landing = 1;
        //     }
        //     ROS_ERROR("landing!!!");
        // }
    }
}
