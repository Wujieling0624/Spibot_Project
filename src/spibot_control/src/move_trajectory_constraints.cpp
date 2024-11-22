#include "move_trajectory_constraints.h"
// # 0-->BR Controllers/1-->FR Controllers/2-->FL Controllers/3-->BL Controllers

std_msgs::Int32 leg_is_moving;
std_msgs::Bool suction1_switch;
std_msgs::Bool suction2_switch;
std_msgs::Bool suction3_switch;
std_msgs::Bool suction4_switch;

std::array<float, 3> BR_Forward_Trajectory(double passTime, int periodCnt)
{
    float BR_traj[3];
    std::array<float, 3> rad;
    BR_traj[1] = y_stretch;
    BR_traj[2] = 0 + z_stretch;
    if (passTime > (periodCnt + 1 / 3.0f) * swingPeriod && passTime <= (periodCnt + 1 / 2.0f) * swingPeriod)
        // ROS_INFO("two");
        BR_traj[0] = (-6.0f * x_stretch / swingPeriod) * (passTime - (periodCnt + 1 / 3.0f) * swingPeriod);
    else if (passTime > (periodCnt + 1 / 2.0f) * swingPeriod && passTime <= (periodCnt + 2 / 3.0f) * swingPeriod)
    {
        leg_is_moving.data = BR_leg;
        suction1_switch.data = true;
        if (passTime > (periodCnt + 1 / 2.0f) * swingPeriod && passTime <= (periodCnt + 1 / 2.0f + 1 / 24.0f) * swingPeriod)
        {
            // ROS_INFO("step1");
            rad[0] = hip_start_rad;
            rad[1] = leg_start_rad + (24.0f / swingPeriod) * (leg_end_rad - leg_start_rad) * (passTime - (periodCnt + 1 / 2.0f) * swingPeriod);
            rad[2] = rad[1];
            rad[1] = -rad[1];
        }
        else if (passTime > (periodCnt + 2 / 3.0f - 1 / 24.0f) * swingPeriod && passTime <= (periodCnt + 2 / 3.0f) * swingPeriod)
        {
            // ROS_INFO("step3");
            rad[0] = hip_end_rad;
            rad[1] = leg_end_rad - (24.0f / swingPeriod) * (leg_end_rad - leg_start_rad) * (passTime - (periodCnt + 1 / 2.0f) * swingPeriod - 3.0f * swingPeriod / 24.0f);
            rad[2] = rad[1];
            rad[1] = -rad[1];
        }
        else
        {
            // ROS_INFO("step2");
            rad[0] = hip_start_rad + (12.0f / swingPeriod) * (hip_end_rad - hip_start_rad) * (passTime - (periodCnt + 1 / 2.0f + 1 / 24.0f) * swingPeriod);
            rad[1] = leg_end_rad;
            rad[2] = rad[1];
            rad[1] = -rad[1];
        }
        return {rad[0], rad[1], rad[2]};
    }
    else if (passTime > (periodCnt + 2 / 3.0f) * swingPeriod && passTime <= (periodCnt + 5 / 6.0f) * swingPeriod)
        // ROS_INFO("three");
        BR_traj[0] = x_stretch;
    else if (passTime > (periodCnt + 5 / 6.0f) * swingPeriod && passTime <= (periodCnt + 1.0f) * swingPeriod)
        // ROS_INFO("four");
        BR_traj[0] = (-6.0f * x_stretch / swingPeriod) * (passTime - (periodCnt + 1.0f) * swingPeriod);
    else
        // ROS_INFO("one");
        BR_traj[0] = 0;
    rad = PosToTheta(BR_traj[0], BR_traj[1], BR_traj[2]);
    rad[1] = -rad[1];
    return rad;
}

std::array<float, 3> BL_Forward_Trajectory(double passTime, int periodCnt)
{
    float BL_traj[3];
    std::array<float, 3> rad;
    BL_traj[1] = y_stretch;
    BL_traj[2] = z_stretch;
    if (passTime > periodCnt * swingPeriod && passTime <= (periodCnt + 1 / 6.0f) * swingPeriod)
    {
        leg_is_moving.data = BL_leg;
        suction4_switch.data = true;
        if (passTime > periodCnt * swingPeriod && passTime <= (periodCnt + 1 / 24.0f) * swingPeriod)
        {
            // ROS_INFO("step1");
            rad[0] = hip_start_rad;
            rad[1] = leg_start_rad + (24.0f / swingPeriod) * (leg_end_rad - leg_start_rad) * (passTime - periodCnt * swingPeriod);
            rad[2] = -rad[1];
        }
        else if (passTime > (periodCnt + 1 / 6.0f - 1 / 24.0f) * swingPeriod && passTime <= (periodCnt + 1 / 6.0f) * swingPeriod)
        {
            // ROS_INFO("step3");
            rad[0] = hip_end_rad;
            rad[1] = leg_end_rad - (24.0f / swingPeriod) * (leg_end_rad - leg_start_rad) * (passTime - (periodCnt + 3 / 24.0f) * swingPeriod);
            rad[2] = -rad[1];
        }
        else
        {
            // ROS_INFO("step2");
            rad[0] = hip_start_rad + (12.0f / swingPeriod) * (hip_end_rad - hip_start_rad) * (passTime - (periodCnt + 1 / 24.0f) * swingPeriod);
            rad[1] = leg_end_rad;
            rad[2] = -rad[1];
        }
        return {rad[0], rad[1], rad[2]};
    }
    else if (passTime > (periodCnt + 1 / 6.0f) * swingPeriod && passTime <= (periodCnt + 1 / 3.0f) * swingPeriod)
        BL_traj[0] = x_stretch;
    else if (passTime > (periodCnt + 1 / 3.0f) * swingPeriod && passTime <= (periodCnt + 1 / 2.0f) * swingPeriod)
        BL_traj[0] = (-6.0f * x_stretch / swingPeriod) * (passTime - (periodCnt + 1 / 2.0f) * swingPeriod);
    else if (passTime > (periodCnt + 1 / 2.0f) * swingPeriod && passTime <= (periodCnt + 5 / 6.0f) * swingPeriod)
        BL_traj[0] = 0;
    else
        BL_traj[0] = (-6.0f * x_stretch / swingPeriod) * (passTime - (periodCnt + 5 / 6.0f) * swingPeriod);
    rad = PosToTheta(BL_traj[0], BL_traj[1], BL_traj[2]);
    rad[2] = -rad[2];
    return rad;
}

std::array<float, 3> FL_Forward_Trajectory(double passTime, int periodCnt)
{
    float FL_traj[3];
    std::array<float, 3> rad;
    FL_traj[1] = y_stretch;
    FL_traj[2] = z_stretch;
    if (passTime > periodCnt * swingPeriod && passTime <= (periodCnt + 1 / 6.0f) * swingPeriod)
        FL_traj[0] = -x_stretch;
    else if (passTime > (periodCnt + 1 / 6.0f) * swingPeriod && passTime <= (periodCnt + 1 / 3.0f) * swingPeriod)
    {
        leg_is_moving.data = FL_leg;
        suction3_switch.data = true;
        if (passTime > (periodCnt + 1 / 6.0f) * swingPeriod && passTime <= (periodCnt + 1 / 6.0f + 1 / 24.0f) * swingPeriod)
        {
            // ROS_INFO("step1");
            rad[0] = hip_start_rad;
            rad[1] = leg_start_rad + (24.0f / swingPeriod) * (leg_end_rad - leg_start_rad) * (passTime - (periodCnt + 1 / 6.0f) * swingPeriod);
            rad[2] = -rad[1];
        }
        else if (passTime > (periodCnt + 1 / 3.0f - 1 / 24.0f) * swingPeriod && passTime <= (periodCnt + 1 / 3.0f) * swingPeriod)
        {
            // ROS_INFO("step3");
            rad[0] = hip_end_rad;
            rad[1] = leg_end_rad - (24.0f / swingPeriod) * (leg_end_rad - leg_start_rad) * (passTime - (periodCnt + 1 / 6.0f + 3 / 24.0f) * swingPeriod);
            rad[2] = -rad[1];
        }
        else
        {
            // ROS_INFO("step2");
            rad[0] = hip_start_rad + (12.0f / swingPeriod) * (hip_end_rad - hip_start_rad) * (passTime - (periodCnt + 1 / 6.0f + 1 / 24.0f) * swingPeriod);
            rad[1] = leg_end_rad;
            rad[2] = -rad[1];
        }
        return {-rad[0], rad[1], rad[2]};
    }
    else if (passTime > (periodCnt + 1 / 3.0f) * swingPeriod && passTime <= (periodCnt + 1 / 2.0f) * swingPeriod)
        FL_traj[0] = (-6.0f * x_stretch / swingPeriod) * (passTime - (periodCnt + 1 / 2.0f) * swingPeriod);
    else if (passTime > (periodCnt + 1 / 2.0f) * swingPeriod && passTime <= (periodCnt + 5 / 6.0f) * swingPeriod)
        FL_traj[0] = 0;
    else
        FL_traj[0] = (-6.0f * x_stretch / swingPeriod) * (passTime - (periodCnt + 5 / 6.0f) * swingPeriod);
    rad = PosToTheta(FL_traj[0], FL_traj[1], FL_traj[2]);
    rad[0] = -rad[0];
    rad[2] = -rad[2];
    return rad;
}

std::array<float, 3> FR_Forward_Trajectory(double passTime, int periodCnt)
{
    float FR_traj[3];
    std::array<float, 3> rad;
    FR_traj[1] = y_stretch;
    FR_traj[2] = z_stretch;
    if (passTime > periodCnt * swingPeriod && passTime <= (periodCnt + 1 / 3.0f) * swingPeriod)
        FR_traj[0] = 0;
    else if (passTime > (periodCnt + 1 / 3.0f) * swingPeriod && passTime <= (periodCnt + 1 / 2.0f) * swingPeriod)
        FR_traj[0] = (-6.0f * x_stretch / swingPeriod) * (passTime - (periodCnt + 1 / 3.0f) * swingPeriod);
    else if (passTime > (periodCnt + 1 / 2.0f) * swingPeriod && passTime <= (periodCnt + 2 / 3.0f) * swingPeriod)
        FR_traj[0] = -x_stretch;
    else if (passTime > (periodCnt + 2 / 3.0f) * swingPeriod && passTime <= (periodCnt + 5 / 6.0f) * swingPeriod)
    {
        leg_is_moving.data = FR_leg;
        suction2_switch.data = true;
        if (passTime > (periodCnt + 2 / 3.0f) * swingPeriod && passTime <= (periodCnt + 2 / 3.0f + 1 / 24.0f) * swingPeriod)
        {
            // ROS_INFO("step1");
            rad[0] = hip_start_rad;
            rad[1] = leg_start_rad + (24.0f / swingPeriod) * (leg_end_rad - leg_start_rad) * (passTime - (periodCnt + 2 / 3.0f) * swingPeriod);
            rad[2] = rad[1];
        }
        else if (passTime > (periodCnt + 5 / 6.0f - 1 / 24.0f) * swingPeriod && passTime <= (periodCnt + 5 / 6.0f) * swingPeriod)
        {
            // ROS_INFO("step3");
            rad[0] = hip_end_rad;
            rad[1] = leg_end_rad - (24.0f / swingPeriod) * (leg_end_rad - leg_start_rad) * (passTime - (periodCnt + 2 / 3.0f + 3 / 24.0f) * swingPeriod);
            rad[2] = rad[1];
        }
        else
        {
            // ROS_INFO("step2");
            rad[0] = hip_start_rad + (12.0f / swingPeriod) * (hip_end_rad - hip_start_rad) * (passTime - (periodCnt + 2 / 3.0f + 1 / 24.0f) * swingPeriod);
            rad[1] = leg_end_rad;
            rad[2] = rad[1];
        }
        return {-rad[0], rad[1], rad[2]};
    }
    else
        FR_traj[0] = (-6.0f * x_stretch / swingPeriod) * (passTime - (periodCnt + 1.0f) * swingPeriod);
    rad = PosToTheta(FR_traj[0], FR_traj[1], FR_traj[2]);
    rad[0] = -rad[0];
    return rad;
}
