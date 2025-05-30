#ifndef MOVE_TRAJECTORY_CONSTRAINS_H
#define MOVE_TRAJECTORY_CONSTRAINS_H

#include "ros/ros.h"
#include <stdio.h>
#include "stdlib.h"
#include <iostream>
#include <array>
#include <cmath>
#include "math.h" // 使用C风格的数学头文件
#include "common/const.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

const float leg_start_rad = 0.5;                      // 腿关节开始摆动角度
const float leg_end_rad = 0.9;                        // 腿关节开始摆动角度
const float hip_start_rad = -0.25;                    // 髋关节开始摆动角度
const float hip_end_rad = 0.25;                       // 髋关节结束摆动角度
const float subPeriod = swingPeriod / 3.0;            // 子周期，用于计算半圆轨迹
const float Rot_subPeriod = swingPeriod / 2.0;        // 旋转子周期，用于计算半圆轨迹
const float L = l1 + l2 * cos(leg_start_rad);         // 垂直情况下俯视图中腿的长度L计算
const float y_stretch = L * cos(hip_start_rad);       // 腿沿y轴延展长度计算
const float x_stretch = L * sin(-hip_start_rad);      // 腿沿x轴延展长度计算
const float z_stretch = l3 - l2 * sin(leg_start_rad); // 腿沿z轴延展长度计算

const float w1 = (2 * pi) / (1.0 * subPeriod);
const float w2 = (2 * pi) / (1.0 * Rot_subPeriod);
float y_bias = 0.2f; // 腿展开多长,禁止改变，否则机器人容易倾斜
float x_bias = 0.1f; // x轴偏移量
float z_bias = 0.2f; // z轴偏移量

extern std_msgs::Int32 leg_is_moving;
extern std_msgs::Bool sucker1_switch;
extern std_msgs::Bool sucker2_switch;
extern std_msgs::Bool sucker3_switch;
extern std_msgs::Bool sucker4_switch;

std::array<float, 3> PosToTheta(float xd, float yd, float zd);
// std::array<float, 3> BR_Forward_Trajectory(double passTime, int periodCnt);
// std::array<float, 3> BL_Forward_Trajectory(double passTime, int periodCnt);
// std::array<float, 3> FL_Forward_Trajectory(double passTime, int periodCnt);
// std::array<float, 3> FR_Forward_Trajectory(double passTime, int periodCnt);

std::array<float, 3> _BR_Forward_Trajectory(double passTime, int periodCnt);
std::array<float, 3> _FR_Forward_Trajectory(double passTime, int periodCnt);
std::array<float, 3> _FL_Forward_Trajectory(double passTime, int periodCnt);
std::array<float, 3> _BL_Forward_Trajectory(double passTime, int periodCnt);

std::array<float, 3> _BR_Backward_Trajectory(double passTime, int periodCnt);
std::array<float, 3> _FR_Backward_Trajectory(double passTime, int periodCnt);
std::array<float, 3> _FL_Backward_Trajectory(double passTime, int periodCnt);
std::array<float, 3> _BL_Backward_Trajectory(double passTime, int periodCnt);

std::array<float, 3> _BR_Rotation_Trajectory(double passTime, int periodCnt);
std::array<float, 3> _FR_Rotation_Trajectory(double passTime, int periodCnt);
std::array<float, 3> _FL_Rotation_Trajectory(double passTime, int periodCnt);
std::array<float, 3> _BL_Rotation_Trajectory(double passTime, int periodCnt);

#endif //
