/***************************************************************************************************************************************
 * 
 * 
 * 实现功能：
 * 1、接收joint_publisher_constrains.cpp发送的四个吸盘开关命令
 * 2、实现/gazebo/apply_body_wrench增加力的服务
 * 3、根据命令决定是否调用和关闭吸盘服务
 * 
 * 
***************************************************************************************************************************************/

#include "ros/ros.h"
#include <string>
#include "std_msgs/Bool.h"
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <std_msgs/Time.h>

void applyBodyWrenchClient(std::string body_name, bool flag)
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    gazebo_msgs::ApplyBodyWrench srv;
    srv.request.body_name = body_name;
    srv.request.reference_frame = "world";
    if (flag)
    {
        srv.request.wrench.force.x = 0;
        srv.request.wrench.force.y = 0;
        srv.request.wrench.force.z = -150;
    }
    else
    {
        srv.request.wrench.force.x = 0;
        srv.request.wrench.force.y = 0;
        srv.request.wrench.force.z = 0;
    }
    srv.request.wrench.torque.x = 0;
    srv.request.wrench.torque.y = 0;
    srv.request.wrench.torque.z = 0;
    srv.request.reference_point.x = 0;
    srv.request.reference_point.y = 0;
    srv.request.reference_point.z = 0;
    srv.request.start_time = ros::Time::now();
    srv.request.duration = ros::Duration(-1.0); // 初始化为负数是一种常见的做法，表示持续时间未定义或无限长
    if (client.call(srv))
    {
        ROS_INFO("%s Service call successful", srv.request.body_name.c_str());
    }
    else
    {
        ROS_ERROR("%s Failed to call service", srv.request.body_name.c_str());
    }
}

void Sucker1CMDCallback(const std_msgs::Bool::ConstPtr &msg)
{
    std::string Sucker1_bodyname = "spibot::foot1";
    if (msg->data)
    {
        ROS_INFO("Sucker1 Received: True");
        applyBodyWrenchClient(Sucker1_bodyname, true);
    }
    else
    {
        ROS_INFO("Sucker1 Received: False");
        applyBodyWrenchClient(Sucker1_bodyname, false);
    }
}

void Sucker2CMDCallback(const std_msgs::Bool::ConstPtr &msg)
{
    std::string Sucker2_bodyname = "spibot::foot2";
    if (msg->data)
    {
        ROS_INFO("Sucker2 Received: True");
        applyBodyWrenchClient(Sucker2_bodyname, true);
    }
    else
    {
        ROS_INFO("Sucker2 Received: False");
        applyBodyWrenchClient(Sucker2_bodyname, false);
    }
}

void Sucker3CMDCallback(const std_msgs::Bool::ConstPtr &msg)
{
    std::string Sucker3_bodyname = "spibot::foot3";
    if (msg->data)
    {
        ROS_INFO("Sucker3 Received: True");
        applyBodyWrenchClient(Sucker3_bodyname, true);
    }
    else
    {
        ROS_INFO("Sucker3 Received: False");
        applyBodyWrenchClient(Sucker3_bodyname, false);
    }
}

void Sucker4CMDCallback(const std_msgs::Bool::ConstPtr &msg)
{
    std::string Sucker4_bodyname = "spibot::foot4";
    if (msg->data)
    {
        ROS_INFO("Sucker4 Received: True");
        applyBodyWrenchClient(Sucker4_bodyname, true);
    }
    else
    {
        ROS_INFO("Sucker4 Received: False");
        applyBodyWrenchClient(Sucker4_bodyname, false);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "apply_body_wrench_client");
    ros::NodeHandle nh;
    ros::Subscriber sucker1_cmd = nh.subscribe("/spibot_gazebo/command/sucker1", 1, Sucker1CMDCallback);
    ros::Subscriber sucker2_cmd = nh.subscribe("/spibot_gazebo/command/sucker2", 1, Sucker2CMDCallback);
    ros::Subscriber sucker3_cmd = nh.subscribe("/spibot_gazebo/command/sucker3", 1, Sucker3CMDCallback);
    ros::Subscriber sucker4_cmd = nh.subscribe("/spibot_gazebo/command/sucker4", 1, Sucker4CMDCallback);
    ros::spin();
    return 0;
}
