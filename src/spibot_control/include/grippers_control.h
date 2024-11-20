#ifndef GRIPPERS_CONTROL_H
#define GRIPPERS_CONTROL_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include <std_msgs/Bool.h>
#include "std_srvs/Empty.h"
#include <iostream>

const int num_grippers_topics = 4;
const char *grippers_name[] = {
    "/spibot_vacuum_gripper1/BR_suction_state",
    "/spibot_vacuum_gripper2/FR_suction_state",
    "/spibot_vacuum_gripper3/FL_suction_state",
    "/spibot_vacuum_gripper4/BL_suction_state"};

// 全局变量，用于控制吸盘的状态
bool BR_gripper1flag = true;
bool FR_gripper2flag = true;
bool FL_gripper3flag = true;
bool BL_gripper4flag = true;

// 服务客户端句柄
ros::ServiceClient gripper1_on_client;
ros::ServiceClient gripper1_off_client;
ros::ServiceClient gripper2_on_client;
ros::ServiceClient gripper2_off_client;
ros::ServiceClient gripper3_on_client;
ros::ServiceClient gripper3_off_client;
ros::ServiceClient gripper4_on_client;
ros::ServiceClient gripper4_off_client;

#endif // GRIPPERS_CONTROL_H
