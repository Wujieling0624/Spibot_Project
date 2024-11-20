#ifndef URDF_TRANSFORMER_H
#define URDF_TRANSFORMER_H

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <iostream>
#include <string>
#include "std_msgs/Int32.h"
#include <unordered_map>
#include <Eigen/Dense>
#include <functional> // 为了 std::hash

// 自定义哈希函数对象，用于 std::pair<std::string, std::string>
struct PairHash
{
    std::size_t operator()(const std::pair<std::string, std::string> &p) const
    {
        // 结合两个字符串的哈希值，这里简单地使用 XOR，但也可以使用其他方法
        std::size_t h1 = std::hash<std::string>{}(p.first);
        std::size_t h2 = std::hash<std::string>{}(p.second);
        return h1 ^ (h2 << 1); // 左移一位是为了减少碰撞的可能性
    }
};
std::unordered_map<std::pair<std::string, std::string>, Eigen::Matrix4d, PairHash, std::equal_to<std::pair<std::string, std::string>>> transformMap; // 使用自定义哈希函数和相等比较器的 unordered_map

// 声明全局变量
extern Eigen::Matrix4d Odom2base_matrix;
extern Eigen::Matrix4d base2hip1_matrix, hip2thigh1_matrix, thigh2shank1_matrix;
extern Eigen::Matrix4d base2hip2_matrix, hip2thigh2_matrix, thigh2shank2_matrix;
extern Eigen::Matrix4d base2hip3_matrix, hip2thigh3_matrix, thigh2shank3_matrix;
extern Eigen::Matrix4d base2hip4_matrix, hip2thigh4_matrix, thigh2shank4_matrix;
extern Eigen::Matrix4d Odom2shank1_matrix, Odom2shank2_matrix, Odom2shank3_matrix, Odom2shank4_matrix;
extern Eigen::Matrix4d shank2foot1_matrix, shank2foot2_matrix, shank2foot3_matrix, shank2foot4_matrix;
extern Eigen::Matrix4d Odom2foot1_matrix, Odom2foot2_matrix, Odom2foot3_matrix, Odom2foot4_matrix;

#endif // URDF_TRANSFORMER_H
