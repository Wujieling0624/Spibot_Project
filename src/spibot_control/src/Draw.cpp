#include "urdf_transformer.h" // 包含头文件
#include "move_trajectory_constraints.h"

// // 声明全局变量
// ros::Publisher triangle_pub;

// void legIsMovingCallback(const std_msgs::Int32::ConstPtr &msg)
// {
//     ROS_INFO("Received leg_is_moving: %d", msg->data);

//     // 定义三个点的坐标
//     std::vector<geometry_msgs::Point> triangle_points;
//     switch (msg->data)
//     {
//     case 0:
//         std::cout << "0\n" << std::endl;
//         triangle_points = triangle_add_data(Odom2foot2_matrix, Odom2foot3_matrix, Odom2foot4_matrix);
//         break;
//     case 1:
//         std::cout << "1\n" << std::endl;
//         triangle_points = triangle_add_data(Odom2foot1_matrix, Odom2foot3_matrix, Odom2foot4_matrix);
//         break;
//     case 2:
//         std::cout << "2\n" << std::endl;
//         triangle_points = triangle_add_data(Odom2foot1_matrix, Odom2foot2_matrix, Odom2foot4_matrix);
//         break;
//     case 3:
//         std::cout << "3\n" << std::endl;
//         triangle_points = triangle_add_data(Odom2foot1_matrix, Odom2foot2_matrix, Odom2foot3_matrix);
//         break;
//     default:
//         std::cout << "4\n" << std::endl;
//         break;
//     }
//     // 发布点
//     for (const auto &p : triangle_points)
//     {
//         triangle_pub.publish(p);
//         std::cout << "Point: x=" << p.x << ", y=" << p.y << ", z=" << p.z << std::endl;
//     }
// }

int main(int argc, char *argv[])
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "triangle_point_publisher");
    ros::NodeHandle nh;
    // 创建发布者
    // triangle_pub = nh.advertise<geometry_msgs::Point>("/Triangle_Point", 10);
    // 循环等待回调
    ros::spin();
    return 0;
}
