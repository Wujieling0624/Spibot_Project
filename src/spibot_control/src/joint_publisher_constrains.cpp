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
std::array<float, 3> BR_rads, BL_rads, FR_rads, FL_rads;
double Rot_t = 0.0, Backward_t = 0.0, Forward_t = 0.0;
int Move_mode, RotCnt, BackwardCnt, ForwardCnt;

// 回调函数，当接收到新消息时会被调用
void moveModeCallback(const std_msgs::Int32::ConstPtr &msg)
{

    switch (msg->data)
    {
    case Forward_mode:
        Move_mode = Forward_mode;
        // ROS_INFO("Forward_mode");
        break;
    case Backward_mode:
        Move_mode = Backward_mode;
        // ROS_INFO("Backward_mode");
        break;
    case Left_mode:
        Move_mode = Left_mode;
        // ROS_INFO("Left_mode");
        break;
    case Right_mode:
        Move_mode = Right_mode;
        // ROS_INFO("Right_mode");
        break;
    default:
        Move_mode = Stop;
        // ROS_INFO("stop");
        break;
    }
}

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
    ros::Publisher sucker1_cmd_pub = nh.advertise<std_msgs::Bool>("/spibot_gazebo/command/sucker1", 10);
    ros::Publisher sucker2_cmd_pub = nh.advertise<std_msgs::Bool>("/spibot_gazebo/command/sucker2", 10);
    ros::Publisher sucker3_cmd_pub = nh.advertise<std_msgs::Bool>("/spibot_gazebo/command/sucker3", 10);
    ros::Publisher sucker4_cmd_pub = nh.advertise<std_msgs::Bool>("/spibot_gazebo/command/sucker4", 10);

    // 定义一个订阅者，监听/spibot_gazebo/move_mode话题，消息类型为std_msgs/Int32，回调函数为moveModeCallback
    ros::Subscriber moveMode_sub = nh.subscribe("/spibot_gazebo/move_mode", 10, moveModeCallback);

    ros::Publisher publishers[num_joint_topics]; // 创建发布者数组
    for (int i = 0; i < num_joint_topics; ++i)
        publishers[i] = nh.advertise<std_msgs::Float64>(joint_names[i], 1);

    ros::Rate loop_rate(50);
    BR_rads = {-pi / 4, -pi / 6, pi / 6}; // 1
    FR_rads = {-pi / 4, pi / 6, pi / 6};  // 2
    FL_rads = {-pi / 4, pi / 6, -pi / 6}; // 3
    BL_rads = {-pi / 4, pi / 6, -pi / 6}; // 4
    while (ros::ok())
    {
        sucker1_switch.data = true; // 每次循环前吸盘开启
        sucker2_switch.data = true; // 每次循环前吸盘开启
        sucker3_switch.data = true; // 每次循环前吸盘开启
        sucker4_switch.data = true; // 每次循环前吸盘开启
        // double time = getTimeSecond();
        // ROS_INFO("passtime = %f", time);
        switch (Move_mode)
        {
        case Forward_mode:
            Forward_t += 0.1;                                 // 计算经历时间
            ForwardCnt = (int)(1.0 * Forward_t / swingPeriod); // 第0,1,2,.....周期
            // 下面代码要用时取消#include "move_trajectory_constraints.h"注释，在其中定义leg_is_moving变量
            BR_rads = _BR_Forward_Trajectory(Forward_t, ForwardCnt);
            FR_rads = _FR_Forward_Trajectory(Forward_t, ForwardCnt);
            FL_rads = _FL_Forward_Trajectory(Forward_t, ForwardCnt);
            BL_rads = _BL_Forward_Trajectory(Forward_t, ForwardCnt);
            ROS_INFO("Forward passtime = %f", Forward_t);
            break;
        case Backward_mode:
            Forward_t += 0.1;                                  // 计算经历时间
            BackwardCnt = (int)(1.0 * Forward_t / swingPeriod); // 第0,1,2,.....周期
            BR_rads = _BR_Backward_Trajectory(Forward_t, BackwardCnt);
            FR_rads = _FR_Backward_Trajectory(Forward_t, BackwardCnt);
            FL_rads = _FL_Backward_Trajectory(Forward_t, BackwardCnt);
            BL_rads = _BL_Backward_Trajectory(Forward_t, BackwardCnt);
            ROS_INFO("Backward passtime = %f", Forward_t);
            break;
        case Left_mode:
            Rot_t += 0.1;                              // 计算经历时间
            RotCnt = (int)(1.0 * Rot_t / swingPeriod); // 第0,1,2,.....周期
            ROS_INFO("Left passtime = %f", Rot_t);
            BR_rads = _BR_Rotation_Trajectory(Rot_t, RotCnt);
            FR_rads = _FR_Rotation_Trajectory(Rot_t, RotCnt);
            FL_rads = _FL_Rotation_Trajectory(Rot_t, RotCnt);
            BL_rads = _BL_Rotation_Trajectory(Rot_t, RotCnt);
            ROS_INFO("Left passtime = %f", Rot_t); 
            break;
        case Right_mode:
            Rot_t += 0.1;                               // 计算经历时间
            RotCnt = (int)(1.0 * Rot_t / swingPeriod); // 第0,1,2,.....周期
            BR_rads = _BR_Rotation_Trajectory(Rot_t, RotCnt);
            FR_rads = _FR_Rotation_Trajectory(Rot_t, RotCnt);
            FL_rads = _FL_Rotation_Trajectory(Rot_t, RotCnt);
            BL_rads = _BL_Rotation_Trajectory(Rot_t, RotCnt);
            ROS_INFO("Right passtime = %f", Rot_t);
            break;
        default:
            // 临时处理，实际吸盘状态应该按照机器人情况定
            sucker1_switch.data = false; // 每次循环前吸盘开启
            sucker2_switch.data = false; // 每次循环前吸盘开启
            sucker3_switch.data = false; // 每次循环前吸盘开启
            sucker4_switch.data = false; // 每次循环前吸盘开启
            break;
        }

        // 下面代码要用时取消#include "baselink_twist.h"注释
        // BR_rads = base_trajectroy(passTime, periodCnt, BR_leg);
        // FR_rads = base_trajectroy(passTime, periodCnt, FR_leg);
        // FL_rads = base_trajectroy(passTime, periodCnt, FL_leg);
        // BL_rads = base_trajectroy(passTime, periodCnt, BL_leg);

        leg_state_pub.publish(leg_is_moving);
        sucker1_cmd_pub.publish(sucker1_switch);
        sucker2_cmd_pub.publish(sucker2_switch);
        sucker3_cmd_pub.publish(sucker3_switch);
        sucker4_cmd_pub.publish(sucker4_switch);
        // ROS_INFO(" Published leg_is_moving: %d \nPublished sucker1_cmd: %d \nPublished sucker2_cmd: %d \nPublished sucker3_cmd: %d \nPublished sucker4_cmd: %d \n  ", leg_is_moving.data, sucker1_switch.data, sucker2_switch.data, sucker3_switch.data, sucker4_switch.data);

        // Leg radian publish
        pubLegRadian(publishers, 0, BR_rads); // BR Controllers
        pubLegRadian(publishers, 1, FR_rads); // FR Controllers
        pubLegRadian(publishers, 2, FL_rads); // FL Controllers
        pubLegRadian(publishers, 3, BL_rads); // BL Controllers

        JointStateUpdate(joint_state_pub, BR_rads, FR_rads, FL_rads, BL_rads);

        loop_rate.sleep();
        ros::spinOnce();
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
