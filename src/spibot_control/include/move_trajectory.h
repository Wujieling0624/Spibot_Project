#ifndef MOVE_TRAJECTORY_H
#define MOVE_TRAJECTORY_H

#include "ros/ros.h"
#include <stdio.h>
#include "stdlib.h"
#include <iostream>
#include <array>
#include <cmath>
#include "math.h" // 使用C风格的数学头文件
#include "common/const.h"

const float link_Stretch = 0.20f;          // 腿展开多长
const float x_offset = 0.15f;              // x轴偏移量
const float z_offset = 0.2f;               // z轴偏移量
const float subPeriod = swingPeriod / 3.0; // 子周期，用于计算半圆轨迹
const float w1 = (2 * pi) / (1.0 * subPeriod);

std::array<float, 3> BR_Forward_Trajectory(double passTime, int periodCnt);
std::array<float, 3> BL_Forward_Trajectory(double passTime, int periodCnt);
std::array<float, 3> FR_Forward_Trajectory(double passTime, int periodCnt);
std::array<float, 3> FL_Forward_Trajectory(double passTime, int periodCnt);

std::array<float, 3> BR_Joint2Theta(std::array<float, 3> traj);
std::array<float, 3> BL_Joint2Theta(std::array<float, 3> traj);
std::array<float, 3> FR_Joint2Theta(std::array<float, 3> traj);
std::array<float, 3> FL_Joint2Theta(std::array<float, 3> traj);

#endif //