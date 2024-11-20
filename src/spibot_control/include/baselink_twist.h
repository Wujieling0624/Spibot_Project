#ifndef BASELINK_TWIST_H
#define BASELINK_TWIST_H

#include "ros/ros.h"
#include <stdio.h>
#include "stdlib.h"
#include <iostream>
#include <array>
#include <cmath>
#include "math.h" // 使用C风格的数学头文件
#include "common/const.h"

const float x_offset = 0.23f; // x轴偏移量
const float y_offset = 0.23f; // y轴偏移量
const float z_offset = 0.3f;  // z轴偏移量

std::array<float, 3> base_trajectroy(double passTime, int periodCnt, LEG_NAME leg_name);
std::array<float, 3> base_updown(double passTime, int periodCnt, LEG_NAME leg_name);
std::array<float, 3> base_twist(double passTime, int periodCnt, LEG_NAME leg_name);

#endif // BASELINK_TWIST_H
