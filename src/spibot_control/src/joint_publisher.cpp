// 发布节点的文件，用于发布机器人模型的状态：joint_publisher，以在Gazebo仿真环境中控制机器人模型的位置和姿态。
// # 0-->BR Controllers/1-->FR Controllers/2-->FL Controllers/3-->BL Controllers
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "move_trajectory.h"
#include <stdio.h>
#include "common/timeMarker.h"

#define num_legs 4
#define num_joint_topics 12 // 关节主题的数量，每条腿3个关节，共4条腿

int legID = 0;
double currentTime, startTime, passTime;
// 定义关节名称和对应的发布者
const char *joint_names[] = {
    "/spibot_gazebo/hip1_controller/command",
    "/spibot_gazebo/thigh1_controller/command",
    "/spibot_gazebo/shank1_controller/command",
    "/spibot_gazebo/hip2_controller/command",
    "/spibot_gazebo/thigh2_controller/command",
    "/spibot_gazebo/shank2_controller/command",
    "/spibot_gazebo/hip3_controller/command",
    "/spibot_gazebo/thigh3_controller/command",
    "/spibot_gazebo/shank3_controller/command",
    "/spibot_gazebo/hip4_controller/command",
    "/spibot_gazebo/thigh4_controller/command",
    "/spibot_gazebo/shank4_controller/command"};

void pubLegRadian(ros::Publisher publishers[], int legID, std::array<float, 3> pub_rads);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_publisher");
    ros::NodeHandle nh;
    ros::Publisher publishers[num_joint_topics]; // 创建发布者数组
    for (int i = 0; i < num_joint_topics; ++i)
    {
        publishers[i] = nh.advertise<std_msgs::Float64>(joint_names[i], 1);
    }

    startTime = getTimeSecond(); // 获取系统开始时间
    ros::Rate loop_rate(50);
    std::array<float, 3> BR_rads = {0, 0, 0};
    std::array<float, 3> FR_rads = {0, 0, 0};
    std::array<float, 3> FL_rads = {0, 0, 0};
    std::array<float, 3> BL_rads = {0, 0, 0};
    while (ros::ok())
    {
        currentTime = getTimeSecond();                       // 系统实际时间
        passTime = currentTime - startTime;                  // 程序开始到现在系统经历时间
        int periodCnt = (int)(1.0 * passTime / swingPeriod); // 第0,1,2,.....周期

        std::array<float, 3> BR_trajs = BR_Forward_Trajectory(passTime, periodCnt);
        std::array<float, 3> BL_trajs = BL_Forward_Trajectory(passTime, periodCnt);
        std::array<float, 3> FR_trajs = FR_Forward_Trajectory(passTime, periodCnt);
        std::array<float, 3> FL_trajs = FL_Forward_Trajectory(passTime, periodCnt);

        BR_rads = BR_Joint2Theta(BR_trajs);
        BL_rads = BL_Joint2Theta(BL_trajs);
        FR_rads = FR_Joint2Theta(FR_trajs);
        FL_rads = FL_Joint2Theta(FL_trajs);

        // ROS_INFO("FL_xd = %f,FL_yd = %f,FL_zd = %f \n", FL_trajs[0], FL_trajs[1], FL_trajs[2]);
        ROS_INFO("hip_theta = %f,thigh_theta = %f,shank_theta = %f \n", FL_rads[0], FL_rads[1], FL_rads[2]);
        // Leg radian publish
        pubLegRadian(publishers, 0, BR_rads); // BR Controllers
        pubLegRadian(publishers, 1, FR_rads); // FR Controllers
        pubLegRadian(publishers, 2, FL_rads); // FL Controllers
        pubLegRadian(publishers, 3, BL_rads); // BL Controllers

        loop_rate.sleep();
    }
    return 0;
}

void pubLegRadian(ros::Publisher publishers[], int legID, std::array<float, 3> pub_rads)
{
    std_msgs::Float64 msgs[num_joint_topics];
    // 检查 legID 是否在有效范围内
    if (legID < 0 || legID >= num_legs)
    {
        std::cerr << "Error: legID out of range" << std::endl;
        return;
    }
    msgs[legID * 3 + 0].data = pub_rads[0];
    msgs[legID * 3 + 1].data = pub_rads[1];
    msgs[legID * 3 + 2].data = pub_rads[2];

    // 统一发布关节角度信息
    publishers[legID * 3 + 0].publish(msgs[legID * 3 + 0]);
    publishers[legID * 3 + 1].publish(msgs[legID * 3 + 1]);
    publishers[legID * 3 + 2].publish(msgs[legID * 3 + 2]);
}
