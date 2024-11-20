#include "urdf_transformer.h" // 包含头文件

// 将geometry_msgs::Transform转换为4x4齐次变换矩阵（Eigen::Matrix4d）
Eigen::Matrix4d transformToEigenMatrix4d(const geometry_msgs::Transform &transform)
{
    // 创建一个4x4的单位矩阵
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    // 将 geometry_msgs::Quaternion 转换为 tf2::Quaternion
    tf2::Quaternion q;
    tf2::fromMsg(transform.rotation, q);
    // 创建一个 tf2::Matrix3x3 对象来存储旋转矩阵
    tf2::Matrix3x3 tf2_rotation_matrix(q);
    // 将 tf2::Matrix3x3 转换为 Eigen::Matrix3d
    Eigen::Matrix3d rotation_matrix;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rotation_matrix(i, j) = tf2_rotation_matrix[i][j];
        }
    }
    // 将旋转矩阵赋值给4x4矩阵的左上角3x3部分
    mat.block<3, 3>(0, 0) = rotation_matrix;
    // 将平移向量赋值给4x4矩阵的最后一列的前三行
    mat(0, 3) = transform.translation.x;
    mat(1, 3) = transform.translation.y;
    mat(2, 3) = transform.translation.z;
    return mat;
}

// 查找特定 parent 和 child 之间的变换矩阵（示例函数）
Eigen::Matrix4d findTransform(const std::string &parent, const std::string &child)
{
    auto it = transformMap.find({parent, child});
    if (it != transformMap.end())
    {
        Eigen::Matrix4d result = it->second;
        return result;
    }
    else
    {
        std::cout << "Transform not found!" << std::endl;
    }
}

// 使用变换矩阵
Eigen::Matrix4d base2hip1_matrix, hip2thigh1_matrix, thigh2shank1_matrix;
Eigen::Matrix4d base2hip2_matrix, hip2thigh2_matrix, thigh2shank2_matrix;
Eigen::Matrix4d base2hip3_matrix, hip2thigh3_matrix, thigh2shank3_matrix;
Eigen::Matrix4d base2hip4_matrix, hip2thigh4_matrix, thigh2shank4_matrix;
// 回调函数实现
void tfCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    for (const auto &tf : msg->transforms)
    {
        std::pair<std::string, std::string> key = {tf.header.frame_id, tf.child_frame_id};
        Eigen::Matrix4d transform_matrix = transformToEigenMatrix4d(tf.transform);
        // 更新或插入变换关系
        transformMap[key] = transform_matrix;
    }
    base2hip1_matrix = findTransform("base_link", "hip1");
    hip2thigh1_matrix = findTransform("hip1", "thigh1");
    thigh2shank1_matrix = findTransform("thigh1", "shank1");
    base2hip2_matrix = findTransform("base_link", "hip2");
    hip2thigh2_matrix = findTransform("hip2", "thigh2");
    thigh2shank2_matrix = findTransform("thigh2", "shank2");
    base2hip3_matrix = findTransform("base_link", "hip3");
    hip2thigh3_matrix = findTransform("hip3", "thigh3");
    thigh2shank3_matrix = findTransform("thigh3", "shank3");
    base2hip4_matrix = findTransform("base_link", "hip4");
    hip2thigh4_matrix = findTransform("hip4", "thigh4");
    thigh2shank4_matrix = findTransform("thigh4", "shank4");
    // 使用 matrix 进行进一步处理
    std::cout << "Found transform:\n"
              << base2hip1_matrix << std::endl;
}

Eigen::Matrix4d Odom2base_matrix;
Eigen::Matrix4d Odom2shank1_matrix, Odom2shank2_matrix, Odom2shank3_matrix, Odom2shank4_matrix;
Eigen::Matrix4d Odom2foot1_matrix, Odom2foot2_matrix, Odom2foot3_matrix, Odom2foot4_matrix;
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    // 创建一个4x4的单位矩阵
    Odom2base_matrix = Eigen::Matrix4d::Identity();
    // 将 geometry_msgs::Quaternion 转换为 tf2::Quaternion
    tf2::Quaternion q;
    tf2::fromMsg(odom->pose.pose.orientation, q);
    // 创建一个 tf2::Matrix3x3 对象来存储旋转矩阵
    tf2::Matrix3x3 tf2_rotation_matrix(q);
    // 将 tf2::Matrix3x3 转换为 Eigen::Matrix3d
    Eigen::Matrix3d rotation_matrix;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rotation_matrix(i, j) = tf2_rotation_matrix[i][j];
        }
    }
    // 将旋转矩阵赋值给4x4矩阵的左上角3x3部分
    Odom2base_matrix.block<3, 3>(0, 0) = rotation_matrix;
    // 将平移向量赋值给4x4矩阵的最后一列的前三行
    Odom2base_matrix(0, 3) = odom->pose.pose.position.x;
    Odom2base_matrix(1, 3) = odom->pose.pose.position.y;
    Odom2base_matrix(2, 3) = odom->pose.pose.position.z;
    // 使用 matrix 进行进一步处理
    Odom2shank1_matrix = Odom2base_matrix * base2hip1_matrix * hip2thigh1_matrix * thigh2shank1_matrix;
    Odom2shank2_matrix = Odom2base_matrix * base2hip2_matrix * hip2thigh2_matrix * thigh2shank2_matrix;
    Odom2shank3_matrix = Odom2base_matrix * base2hip3_matrix * hip2thigh3_matrix * thigh2shank3_matrix;
    Odom2shank4_matrix = Odom2base_matrix * base2hip4_matrix * hip2thigh4_matrix * thigh2shank4_matrix;

    Odom2foot1_matrix = Odom2shank1_matrix * shank2foot1_matrix;
    Odom2foot2_matrix = Odom2shank2_matrix * shank2foot2_matrix;
    Odom2foot3_matrix = Odom2shank3_matrix * shank2foot3_matrix;
    Odom2foot4_matrix = Odom2shank4_matrix * shank2foot4_matrix;
    std::cout << "Odom2foot1_matrix:\n"
              << Odom2foot1_matrix << std::endl;
}

Eigen::Matrix4d shank2foot1_matrix, shank2foot2_matrix, shank2foot3_matrix, shank2foot4_matrix;
void tfstatic_Callback(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    for (const auto &tf : msg->transforms)
    {
        std::pair<std::string, std::string> key = {tf.header.frame_id, tf.child_frame_id};
        Eigen::Matrix4d transform_matrix = transformToEigenMatrix4d(tf.transform);
        // 更新或插入变换关系
        transformMap[key] = transform_matrix;
    }
    shank2foot1_matrix = findTransform("shank1", "foot1");
    shank2foot2_matrix = findTransform("shank2", "foot2");
    shank2foot3_matrix = findTransform("shank3", "foot3");
    shank2foot4_matrix = findTransform("shank4", "foot4");
    // 使用 matrix 进行进一步处理
    std::cout << "Found transform:\n"
              << shank2foot4_matrix << std::endl;
}

void legIsMovingCallback(const std_msgs::Int32::ConstPtr &msg)
{
    ROS_INFO("Received leg_is_moving: %d", msg->data);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "matrix_creator");
    ros::NodeHandle nh;

    // 订阅 /tf 话题
    ros::Subscriber robot_sub = nh.subscribe("/tf", 100, tfCallback);
    // 订阅 /tf_static 话题
    ros::Subscriber robot_static_sub = nh.subscribe("/tf_static", 100, tfstatic_Callback);
    // 订阅 /Odometry 话题，并传递回调函数指针
    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 100, odomCallback);
    // 订阅 /leg_is_moving 话题
    ros::Subscriber leg_state_sub = nh.subscribe("/leg_is_moving", 10, legIsMovingCallback);

    ros::spin();
    return 0;
}
