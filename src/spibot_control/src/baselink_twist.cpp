/***************************************************************************************************************************************
 * 
 * 
 * 定义机器人基坐标系运动函数
 * base_trajectroy函数：实现机器人沿x方向前后移动
 * base_updown函数：实现机器人沿z方向前后移动
 * base_twist函数，实现机器人沿x轴旋转
 * 其他功能参考三个函数自行定义
 * 
 * 
***************************************************************************************************************************************/


#include "baselink_twist.h"

std::array<float, 3> base_trajectroy(double passTime, int periodCnt, LegName leg_name)
{
    float traj[3];
    std::array<float, 3> rad;
    if (passTime > periodCnt * swingPeriod && passTime <= (periodCnt + 1 / 2.0f) * swingPeriod)
        traj[0] = 0.1 - 2.0 * swingRaduis * (passTime - periodCnt * swingPeriod) / swingPeriod;
    else
        traj[0] = 0.1 + 2.0 * swingRaduis * (passTime - (periodCnt + 1) * swingPeriod) / swingPeriod;
    traj[1] = y_offset;
    traj[2] = z_offset;
    rad = PosToTheta(traj[0], traj[1], traj[2]);
    rad = Leg_decision(leg_name, rad);
    return rad;
}

std::array<float, 3> base_updown(double passTime, int periodCnt, LegName leg_name)
{
    float traj[3];
    std::array<float, 3> rad;
    switch (leg_name)
    {
    case BR_leg:
    case BL_leg:
        traj[0] = -x_offset;
        break;
    case FR_leg:
    case FL_leg:
        traj[0] = x_offset;
        break;
    }
    if (passTime > periodCnt * swingPeriod && passTime <= (periodCnt + 1 / 2.0f) * swingPeriod)
        traj[2] = z_offset - 2.0 * swingRaduis * (passTime - periodCnt * swingPeriod) / swingPeriod;
    else
        traj[2] = z_offset + 2.0 * swingRaduis * (passTime - (periodCnt + 1) * swingPeriod) / swingPeriod;
    traj[1] = y_offset;
    rad = PosToTheta(traj[0], traj[1], traj[2]);
    rad = Leg_decision(leg_name, rad);
    return rad;
}

std::array<float, 3> base_twist(double passTime, int periodCnt, LegName leg_name)
{
    float traj[3];
    Eigen::Vector3d pos{0.0, y_offset, z_offset};
    std::array<float, 3> rad;
    if (leg_name == BR_leg || leg_name == BL_leg)
        pos[0] = -x_offset;
    else
        pos[0] = x_offset;
    float rad_x = pi / 8;
    float rot_rad = 0;
    if (passTime > periodCnt * swingPeriod && passTime <= (periodCnt + 1 / 2.0f) * swingPeriod)
        rot_rad = -2.0 * rad_x * (passTime - periodCnt * swingPeriod) / swingPeriod;
    else
        rot_rad = 2.0 * rad_x * (passTime - (periodCnt + 1) * swingPeriod) / swingPeriod;
    if (leg_name == BR_leg || leg_name == FR_leg)
        rot_rad = -rot_rad;
    Eigen::Matrix3d R01 = rotationMatrixX(rot_rad);
    std::cout << "R01 Matrix:\n"
              << R01 << std::endl;
    Eigen::Vector3d new_pos;
    Eigen::Matrix3d R10 = R01.inverse();
    new_pos = R10 * pos;
    rad = PosToTheta(new_pos[0], new_pos[1], new_pos[2]);
    rad = Leg_decision(leg_name, rad);
    return rad;
}
