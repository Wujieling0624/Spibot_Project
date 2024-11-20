#include "grippers_control.h"

void gripper_status_callback(const std_msgs::Bool::ConstPtr &msg)
{
    ROS_INFO("Gripper status = %d", msg->data);
}

// 定义一个函数来处理单个吸盘的状态
void handleGripper(bool &flag, ros::ServiceClient &onClient, ros::ServiceClient &offClient, const std::string &gripperName)
{
    std_srvs::Empty srv;
    if (flag)
    {
        if (onClient.call(srv))
        {
            ROS_INFO("%s turned on", gripperName.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service %s_on", gripperName.c_str());
        }
    }
    else
    {
        if (offClient.call(srv))
        {
            ROS_INFO("%s turned off", gripperName.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service %s_off", gripperName.c_str());
        }
    }
}
// 定时器回调函数
void timerCallback(const ros::TimerEvent &)
{
    // 使用函数来处理每个吸盘的状态
    handleGripper(BR_gripper1flag, gripper1_on_client, gripper1_off_client, "Gripper1");
    handleGripper(FR_gripper2flag, gripper2_on_client, gripper2_off_client, "Gripper2");
    handleGripper(FL_gripper3flag, gripper3_on_client, gripper3_off_client, "Gripper3");
    handleGripper(BL_gripper4flag, gripper4_on_client, gripper4_off_client, "Gripper4");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grippers_controller");
    ros::NodeHandle nh;

    gripper1_on_client = nh.serviceClient<std_srvs::Empty>("/spibot_vacuum_gripper1/on");   // BR_gripper1开启
    gripper1_off_client = nh.serviceClient<std_srvs::Empty>("/spibot_vacuum_gripper1/off"); // BR_gripper1关闭
    gripper2_on_client = nh.serviceClient<std_srvs::Empty>("/spibot_vacuum_gripper2/on");   // BR_gripper2开启
    gripper2_off_client = nh.serviceClient<std_srvs::Empty>("/spibot_vacuum_gripper2/off"); // BR_gripper2关闭
    gripper3_on_client = nh.serviceClient<std_srvs::Empty>("/spibot_vacuum_gripper3/on");   // BR_gripper3开启
    gripper3_off_client = nh.serviceClient<std_srvs::Empty>("/spibot_vacuum_gripper3/off"); // BR_gripper3关闭
    gripper4_on_client = nh.serviceClient<std_srvs::Empty>("/spibot_vacuum_gripper4/on");   // BR_gripper4开启
    gripper4_off_client = nh.serviceClient<std_srvs::Empty>("/spibot_vacuum_gripper4/off"); // BR_gripper4关闭

    ros::Subscriber subscribers[num_grippers_topics];
    for (int i = 0; i < num_grippers_topics; ++i)
    {
        subscribers[i] = nh.subscribe(grippers_name[i], 1, gripper_status_callback);
    }
    // 定时器，用于检查全局变量并控制吸盘
    ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback);

    ros::spin(); // 保持程序运行并处理回调
    return 0;
}
