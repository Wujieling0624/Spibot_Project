#include "urdf_transformer.h" // 包含头文件
#include "move_trajectory_constraints.h"

int main(int argc, char *argv[])
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "triangle_point_publisher");
    ros::NodeHandle nh;
    // 循环等待回调
    ros::spin();
    return 0;
}
