#ifndef CONST_H
#define CONST_H

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "math.h" // 使用C风格的数学头文件

const float pi = 3.14159f;
const double l1 = 0.07;          // 第一个连杆长度（hip到thigh）
const double l2 = 0.23;          // 第二个连杆长度（thigh到shank）
const float l3 = 0.36572;        // 第三个连杆长度（shank末端到关节）
const int swingPeriod = 6;       // 摆腿周期
const float swingRaduis = 0.15f; // 摆腿半径

// 定义一个枚举类型表示机器人的腿
enum LegName
{
    None,   // 单纯为了凑数
    BR_leg, // 左前腿
    FR_leg, // 左后腿
    FL_leg, // 右前腿
    BL_leg  // 右后腿
};

// 定义一个枚举类型表示机器人的运动类型
enum MoveMode
{
    Stop,          // 机器人停止
    Forward_mode,  // 前进模式
    Backward_mode, // 后退模式
    Left_mode,     // 向左转
    Right_mode     // 向右转
};

std::array<float, 3> PosToTheta(float xd, float yd, float zd)
{
    // 目标角度计算
    float L, rad1, ar, Lr, a1, a2, rad2, rad3;
    L = sqrt(xd * xd + yd * yd);
    rad1 = atan2(xd, yd);   // 弧度
    ar = atan2(zd, L - l1); // 弧度
    Lr = sqrt(zd * zd + (L - l1) * (L - l1));
    a1 = acos((Lr * Lr + l2 * l2 - l3 * l3) / (2.0 * l2 * Lr)); // 弧度
    a2 = acos((Lr * Lr + l3 * l3 - l2 * l2) / (2.0 * Lr * l3)); // 弧度
    rad2 = a1 - ar;                                             // 弧度
    rad3 = -0.5f * pi + a1 + a2;                                // 弧度，注意这里使用了浮点数除法
    return {rad1, rad2, rad3};                                  // 返回结果
}

// 沿x轴旋转的旋转矩阵函数
Eigen::Matrix3d rotationMatrixX(double rad)
{
    Eigen::Matrix3d R;
    R << 1, 0, 0,
        0, std::cos(rad), -std::sin(rad),
        0, std::sin(rad), std::cos(rad);
    return R;
}

// 沿y轴旋转的旋转矩阵函数
Eigen::Matrix3d rotationMatrixY(double rad)
{
    Eigen::Matrix3d R;
    R << std::cos(rad), 0, std::sin(rad),
        0, 1, 0,
        -std::sin(rad), 0, std::cos(rad);
    return R;
}

// 沿z轴旋转的旋转矩阵函数
Eigen::Matrix3d rotationMatrixZ(double rad)
{
    Eigen::Matrix3d R;
    R << std::cos(rad), -std::sin(rad), 0,
        std::sin(rad), std::cos(rad), 0,
        0, 0, 1;
    return R;
}

std::array<float, 3> Leg_decision(LegName leg_name, std::array<float, 3> rad)
{
    switch (leg_name)
    {
    case BR_leg:
        rad[1] = -rad[1];
        break;
    case FR_leg:
        rad[0] = -rad[0];
        break;
    case FL_leg:
        rad[0] = -rad[0];
        rad[2] = -rad[2];
        break;
    case BL_leg:
        rad[2] = -rad[2];
        break;
    default:
        std::cerr << "Wrong leg name!" << std::endl;
    }
    return rad;
}

#endif // CONST_H
