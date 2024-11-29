#include "move_trajectory.h"
// # 0-->BR Controllers/1-->FR Controllers/2-->FL Controllers/3-->BL Controllers

std::array<float, 3> BR_Forward_Trajectory(double passTime, int periodCnt)
{
  float BR_xd, BR_yd, BR_zd;
  BR_yd = link_Stretch; // 沿直线前后摆动
  if (passTime > (periodCnt + 1 / 6.0f) * swingPeriod && passTime <= (periodCnt + 1 / 3.0f) * swingPeriod)
  {
    BR_xd = (-6.0f * swingRaduis / swingPeriod) * (passTime - swingPeriod / 6.0f - periodCnt * swingPeriod) - x_offset;
    BR_zd = 0 + z_offset;
  }
  else if (passTime > (periodCnt + 1 / 3.0f) * swingPeriod && passTime <= (periodCnt + 1 / 2.0f) * swingPeriod)
  {
    BR_xd = -swingRaduis * cos(w1 * passTime) - x_offset;
    BR_yd = link_Stretch + 0.05f * sin(w1 * passTime);
    BR_zd = -swingRaduis * sin(w1 * passTime) + z_offset;
  }
  else if (passTime > (periodCnt + 1 / 2.0f) * swingPeriod && passTime <= (periodCnt + 2 / 3.0f) * swingPeriod)
  {
    BR_xd = swingRaduis - x_offset;
    BR_zd = 0 + z_offset;
  }
  else if (passTime > (periodCnt + 2 / 3.0f) * swingPeriod && passTime <= (periodCnt + 5 / 6.0f) * swingPeriod)
  {
    BR_xd = (-6.0f * swingRaduis / swingPeriod) * (passTime - 5 * swingPeriod / 6.0f - periodCnt * swingPeriod) - x_offset;
    BR_zd = 0 + z_offset;
  }
  else
  {
    BR_xd = 0 - x_offset;
    BR_zd = 0 + z_offset;
  }
  return {BR_xd, BR_yd, BR_zd};
}

std::array<float, 3> FR_Forward_Trajectory(double passTime, int periodCnt)
{
  float FR_xd, FR_yd, FR_zd;
  FR_yd = link_Stretch; // 沿直线前后摆动
  if (passTime > (periodCnt + 1 / 6.0f) * swingPeriod && passTime <= (periodCnt + 1 / 3.0f) * swingPeriod)
  {
    FR_xd = (-6.0f * swingRaduis / swingPeriod) * (passTime - (1 / 6.0f + periodCnt) * swingPeriod) + x_offset;
    FR_zd = 0 + z_offset;
  }
  else if (passTime > (periodCnt + 1 / 3.0f) * swingPeriod && passTime <= (periodCnt + 1 / 2.0f) * swingPeriod)
  {
    FR_xd = -swingRaduis + x_offset;
    FR_zd = 0 + z_offset;
  }
  else if (passTime > (periodCnt + 1 / 2.0f) * swingPeriod && passTime <= (periodCnt + 2 / 3.0f) * swingPeriod)
  {
    FR_xd = swingRaduis * cos(w1 * passTime) + x_offset;
    FR_yd = link_Stretch - 0.05f * sin(w1 * passTime);
    FR_zd = swingRaduis * sin(w1 * passTime) + z_offset;
  }
  else if (passTime > (periodCnt + 2 / 3.0f) * swingPeriod && passTime <= (periodCnt + 5 / 6.0f) * swingPeriod)
  {
    FR_xd = (-6.0f * swingRaduis / swingPeriod) * (passTime - (5 / 6.0f + periodCnt) * swingPeriod) + x_offset;
    FR_zd = 0 + z_offset;
  }
  else
  {
    FR_xd = 0 + x_offset;
    FR_zd = 0 + z_offset;
  }
  return {FR_xd, FR_yd, FR_zd};
}

std::array<float, 3> FL_Forward_Trajectory(double passTime, int periodCnt)
{
  float FL_xd, FL_yd, FL_zd;
  FL_yd = link_Stretch; // 沿直线前后摆动
  if (passTime > 1.0f * periodCnt * swingPeriod && passTime <= (periodCnt + 1 / 6.0f) * swingPeriod)
  {
    FL_xd = -swingRaduis * cos(w1 * passTime) + x_offset;
    FL_yd = link_Stretch + 0.05f * sin(w1 * passTime);
    FL_zd = -swingRaduis * sin(w1 * passTime) + z_offset;
  }
  else if (passTime > (periodCnt + 1 / 6.0f) * swingPeriod && passTime <= (periodCnt + 1 / 3.0f) * swingPeriod)
  {
    FL_xd = (-6.0f * swingRaduis / swingPeriod) * (passTime - swingPeriod / 3.0f - periodCnt * swingPeriod) + x_offset;
    FL_zd = 0 + z_offset;
  }
  else if (passTime > (periodCnt + 1 / 3.0f) * swingPeriod && passTime <= (periodCnt + 2 / 3.0f) * swingPeriod)
  {
    FL_xd = 0 + x_offset;
    FL_zd = 0 + z_offset;
  }
  else if (passTime > (periodCnt + 2 / 3.0f) * swingPeriod && passTime <= (periodCnt + 5 / 6.0f) * swingPeriod)
  {
    FL_xd = (-6.0f * swingRaduis / swingPeriod) * (passTime - 2 * swingPeriod / 3.0f - periodCnt * swingPeriod) + x_offset;
    FL_zd = 0 + z_offset;
  }
  else
  {
    FL_xd = -swingRaduis + x_offset;
    FL_zd = 0 + z_offset;
  }
  return {FL_xd, FL_yd, FL_zd};
}

std::array<float, 3> BL_Forward_Trajectory(double passTime, int periodCnt)
{
  float BL_xd, BL_yd, BL_zd;
  BL_yd = link_Stretch; // 沿直线前后摆动
  if (passTime > 1.0f * periodCnt * swingPeriod && passTime <= (periodCnt + 1 / 6.0f) * swingPeriod)
  {
    BL_xd = swingRaduis - x_offset;
    BL_zd = 0 + z_offset;
  }
  else if (passTime > (periodCnt + 1 / 6.0f) * swingPeriod && passTime <= (periodCnt + 1 / 3.0f) * swingPeriod)
  {
    BL_xd = (-6.0f * swingRaduis / swingPeriod) * (passTime - swingPeriod / 3.0f - periodCnt * swingPeriod) - x_offset;
    BL_zd = 0 + z_offset;
  }
  else if (passTime > (periodCnt + 2 / 3.0f) * swingPeriod && passTime <= (periodCnt + 5 / 6.0f) * swingPeriod)
  {
    BL_xd = (-6.0f * swingRaduis / swingPeriod) * (passTime - 2 * swingPeriod / 3.0f - periodCnt * swingPeriod) - x_offset;
    BL_zd = 0 + z_offset;
  }
  else if (passTime > (periodCnt + 5 / 6.0f) * swingPeriod && passTime <= (periodCnt + 1.0f) * swingPeriod)
  {
    BL_xd = swingRaduis * cos(w1 * passTime) - x_offset;
    BL_yd = link_Stretch - 0.05f * sin(w1 * passTime);
    BL_zd = swingRaduis * sin(w1 * passTime) + z_offset;
  }
  else
  {
    BL_xd = 0 - x_offset;
    BL_zd = 0 + z_offset;
  }
  return {BL_xd, BL_yd, BL_zd};
}

std::array<float, 3> BR_Joint2Theta(std::array<float, 3> traj)
{
  float BR_rad0, BR_rad1, BR_rad2;
  std::array<float, 3> raw_rads = PosToTheta(traj[0], traj[1], traj[2]);
  BR_rad0 = raw_rads[0];
  BR_rad1 = -raw_rads[1];
  BR_rad2 = raw_rads[2];
  return {BR_rad0, BR_rad1, BR_rad2};
}

std::array<float, 3> FR_Joint2Theta(std::array<float, 3> traj)
{
  float FR_rad0, FR_rad1, FR_rad2;
  std::array<float, 3> raw_rads = PosToTheta(traj[0], traj[1], traj[2]);
  FR_rad0 = -raw_rads[0];
  FR_rad1 = raw_rads[1];
  FR_rad2 = raw_rads[2];
  return {FR_rad0, FR_rad1, FR_rad2};
}

std::array<float, 3> FL_Joint2Theta(std::array<float, 3> traj)
{
  float FL_rad0, FL_rad1, FL_rad2;
  std::array<float, 3> raw_rads = PosToTheta(traj[0], traj[1], traj[2]);
  FL_rad0 = -raw_rads[0];
  FL_rad1 = raw_rads[1];
  FL_rad2 = -raw_rads[2];
  return {FL_rad0, FL_rad1, FL_rad2};
}

std::array<float, 3> BL_Joint2Theta(std::array<float, 3> traj)
{
  float BL_rad0, BL_rad1, BL_rad2;
  std::array<float, 3> raw_rads = PosToTheta(traj[0], traj[1], traj[2]);
  BL_rad0 = raw_rads[0];
  BL_rad1 = raw_rads[1];
  BL_rad2 = -raw_rads[2];
  return {BL_rad0, BL_rad1, BL_rad2};
}
