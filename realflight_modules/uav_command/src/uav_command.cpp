#include <ros/ros.h>
#include <uav_command/uav_task.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"

/*FSM of this planner

     system_sart
          |
          |
          v
     Manual_Ctrl
          |
          |
          v
     Auto_takeoff
          |
          |
          v
      Auto_Hover------->Auto_Land
        |   ^           ^
        |   |          /
        v   |         /
      CMD_CTRL(task)-/

*/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_command");

    ros::NodeHandle n;
    UAV_plan::uav_command uav_cmd;
    uav_cmd.init(n);
  
    // uav_cmd发布各个指令，给ego发送2D Nav Goal，给px4ctrl发送pos_cmd和takeoff指令
    uav_cmd.ego_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10); // ego接收，
    uav_cmd.TakeoffLand_pub = n.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
    uav_cmd.px4_pub = n.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10); // px4和traj_server同一个话题，都是发布
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/global_odom", // /vins_fusion/imu_propagate
                                                               1,
                                                               boost::bind(&UAV_plan::odom_data::feed, &uav_cmd.rc_odom, _1));

    // 初始化接收视觉计算位置的客户端，消息类型和话题可改
    //uav_cmd.vision_cylinder_client = n.serviceClient<vision_server::cylinder>("/vision_cylinder_pos");
    uav_cmd.vision_circle_client = n.serviceClient<vision_server::circle>("/vision_circle_pos");
    uav_cmd.vision_switch_mode_client = n.serviceClient<vision_server::switch_mode>("/vision_switch_mode");
    // create Publishers and subscribers.
    // it takes time of register of publishers and subscribers. so, add a delay for 0.5s .
    ros::Duration(0.5).sleep(); // important
    // uav_cmd.task_flag=uav_cmd.takeoff;
    ros::Rate loop_rate(10);//10
    
    while (ros::ok()) // 20 hz
    {
        ros::spinOnce();

        // uav_cmd.ego(3.0, 3.0, 1.0, 0,0.5);

        // uav_cmd.px4(3, 3, 1, 1.57);

        // uav_cmd.go2target(3, 3, 1, 1.57);

        uav_cmd.task_planner();

        loop_rate.sleep();
    }

    return 0;
}
