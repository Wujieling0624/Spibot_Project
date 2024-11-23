// 发布节点的文件，用于发布机器人模型的状态：joint_publisher_constrains，以在Gazebo仿真环境中控制机器人模型的位置和姿态。
// # 0-->BR Controllers/1-->FR Controllers/2-->FL Controllers/3-->BL Controllers
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "move_trajectory_constraints.h"
#include "baselink_twist.h"
#include <vector>
#include <string>
#include "common/timeMarker.h"

const int num_legs = 4;
const int num_joint_topics = 12; // 关节主题的数量，每条腿3个关节，共4条腿

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
void JointStateUpdate(ros::Publisher joint_state_pub, std::array<float, 3> BR_rads, std::array<float, 3> FR_rads, std::array<float, 3> FL_rads, std::array<float, 3> BL_rads);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spibot_move_publisher");
    ros::NodeHandle nh;

    ros::Publisher leg_state_pub = nh.advertise<std_msgs::Int32>("/spibot_gazebo/states/leg_is_moving", 10);
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10); // 创建一个发布者，发布到/joint_state话题
    ros::Publisher suction1_state_pub = nh.advertise<std_msgs::Bool>("/spibot_gazebo/command/suction1", 10);
    ros::Publisher suction2_state_pub = nh.advertise<std_msgs::Bool>("/spibot_gazebo/command/suction2", 10);
    ros::Publisher suction3_state_pub = nh.advertise<std_msgs::Bool>("/spibot_gazebo/command/suction3", 10);
    ros::Publisher suction4_state_pub = nh.advertise<std_msgs::Bool>("/spibot_gazebo/command/suction4", 10);

    ros::Publisher publishers[num_joint_topics]; // 创建发布者数组
    for (int i = 0; i < num_joint_topics; ++i)
        publishers[i] = nh.advertise<std_msgs::Float64>(joint_names[i], 1);
    startTime = getTimeSecond(); // 获取系统开始时间
    ros::Rate loop_rate(100);
    std::array<float, 3> BR_rads = {-0, 0, 0}; // 1
    std::array<float, 3> FR_rads = {-0, 0, 0}; // 2
    std::array<float, 3> FL_rads = {-0, 0, 0}; // 3
    std::array<float, 3> BL_rads = {-0, 0, 0}; // 4
    while (ros::ok())
    {

        currentTime = getTimeSecond();                       // 系统实际时间
        passTime = currentTime - startTime;                  // 程序开始到现在系统经历时间
        int periodCnt = (int)(1.0 * passTime / swingPeriod); // 第0,1,2,.....周期

        suction1_switch.data = true; // 每次循环前吸盘开启
        suction2_switch.data = true; // 每次循环前吸盘开启
        suction3_switch.data = true; // 每次循环前吸盘开启
        suction4_switch.data = true; // 每次循环前吸盘开启
        
        // BR_rads = base_trajectroy(passTime, periodCnt, BR_leg);
        // FR_rads = base_trajectroy(passTime, periodCnt, FR_leg);
        // FL_rads = base_trajectroy(passTime, periodCnt, FL_leg);
        // BL_rads = base_trajectroy(passTime, periodCnt, BL_leg);

        // 下面代码要用时取消include注释，在其中定义leg_is_moving变量
        BR_rads = BR_Forward_Trajectory(passTime, periodCnt);
        FR_rads = FR_Forward_Trajectory(passTime, periodCnt);
        FL_rads = FL_Forward_Trajectory(passTime, periodCnt);
        BL_rads = BL_Forward_Trajectory(passTime, periodCnt);

        // BR_rads = {-0, 0, 0}; // 1
        // FR_rads = {-0, 0, 0}; // 2
        // FL_rads = {-0, 0, 0}; // 3
        // BL_rads = {-0, 0, 0}; // 4
        // ROS_INFO("hip_theta = %f,thigh_theta = %f,shank_theta = %f \n", BR_rads[0], BR_rads[1], BR_rads[2]);

        leg_state_pub.publish(leg_is_moving);
        suction1_state_pub.publish(suction1_switch);
        suction2_state_pub.publish(suction2_switch);
        suction3_state_pub.publish(suction3_switch);
        suction4_state_pub.publish(suction4_switch);
        ROS_INFO(" Published leg_is_moving: %d \n ", leg_is_moving.data);
        ROS_INFO(" Published suction1_cmd: %d \n ", suction1_switch.data);
        ROS_INFO(" Published suction2_cmd: %d \n ", suction2_switch.data);
        ROS_INFO(" Published suction3_cmd: %d \n ", suction3_switch.data);
        ROS_INFO(" Published suction4_cmd: %d \n ", suction4_switch.data);

        // Leg radian publish
        pubLegRadian(publishers, 0, BR_rads); // BR Controllers
        pubLegRadian(publishers, 1, FR_rads); // FR Controllers
        pubLegRadian(publishers, 2, FL_rads); // FL Controllers
        pubLegRadian(publishers, 3, BL_rads); // BL Controllers

        JointStateUpdate(joint_state_pub, BR_rads, FR_rads, FL_rads, BL_rads);

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

// rviz更新实时角度
void JointStateUpdate(ros::Publisher joint_state_pub, std::array<float, 3> BR_rads, std::array<float, 3> FR_rads, std::array<float, 3> FL_rads, std::array<float, 3> BL_rads)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = ""; // 或者设置为合适的坐标帧ID

    std::vector<std::string> joint_names = {
        "base2hip1", "hip2thigh1", "thigh2shank1",
        "base2hip2", "hip2thigh2", "thigh2shank2",
        "base2hip3", "hip2thigh3", "thigh2shank3",
        "base2hip4", "hip2thigh4", "thigh2shank4",
        "shank2foot1", "shank2foot2", "shank2foot3", "shank2foot4"};

    std::vector<double> joint_positions = {
        BR_rads[0], BR_rads[1], BR_rads[2],
        FR_rads[0], FR_rads[1], FR_rads[2],
        FL_rads[0], FL_rads[1], FL_rads[2],
        BL_rads[0], BL_rads[1], BL_rads[2],
        0.0, 0.0, 0.0, 0.0}; // 假设foot关节的角度始终为0

    joint_state.name = joint_names;
    joint_state.position = joint_positions;
    joint_state.velocity.clear();
    joint_state.effort.clear();

    joint_state_pub.publish(joint_state);
}
