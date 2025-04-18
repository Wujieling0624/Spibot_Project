
/***************************************************************************************************************************************
 *
 * 实现一些数据的发布和接收：
 * 订阅机器人rviz节点发布的/tf和/tf_static话题，处理成各个机器人关节间的矩阵
 * 订阅由插件发布的/spibot_gazebo/odometry话题，处理成机器人绝对位姿的矩阵
 * 根据这些矩阵得到仿真环境原点相对于机器人末端坐标系的位置x,y,z
 * 将静止非摆动的三条腿末端的x,y,z点发送到/spibot_gazebo/draw/triangle_points话题中
 *
 * ps:
 * 自定义的绘制三角形插件需要接收/spibot_gazebo/draw/triangle_points话题才能绘制
 *
 ***************************************************************************************************************************************/

#include "urdf_transformer.h" // 包含头文件

// 声明全局变量
ros::Publisher triangle_pub;
// 三个点的坐标，回调函数legIsMovingCallback用
std_msgs::Float32MultiArray triangle_points;

// 查找特定 parent 和 child 之间的变换矩阵（示例函数）
Eigen::Matrix4d findTransform(const std::string &parent, const std::string &child)
{
    auto it = transformMap.find({parent, child});
    if (it != transformMap.end())
    {
        return it->second;
    }
    else
    {
        std::cerr << "Transform not found for: " << parent << " -> " << child << std::endl;
        // return Eigen::Matrix4d::Identity(); // 返回单位矩阵
    }
}

// 提取Odom2foot1_matrix\Odom2foot2_matrix\Odom2foot3_matrix\Odom2foot4_matrix第4列的前3行数据
std_msgs::Float32MultiArray triangle_add_data(Eigen::Matrix4d matrix1, Eigen::Matrix4d matrix2, Eigen::Matrix4d matrix3)
{
    std_msgs::Float32MultiArray points;
    points.data.reserve(9); // 预留空间
    points.data.push_back(matrix1(0, 3));
    points.data.push_back(matrix1(1, 3));
    points.data.push_back(matrix1(2, 3));
    points.data.push_back(matrix2(0, 3));
    points.data.push_back(matrix2(1, 3));
    points.data.push_back(matrix2(2, 3));
    points.data.push_back(matrix3(0, 3));
    points.data.push_back(matrix3(1, 3));
    points.data.push_back(matrix3(2, 3));
    return points;
}

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

// 使用变换矩阵
Eigen::Matrix4d base2hip1_matrix, hip2thigh1_matrix, thigh2shank1_matrix;
Eigen::Matrix4d base2hip2_matrix, hip2thigh2_matrix, thigh2shank2_matrix;
Eigen::Matrix4d base2hip3_matrix, hip2thigh3_matrix, thigh2shank3_matrix;
Eigen::Matrix4d base2hip4_matrix, hip2thigh4_matrix, thigh2shank4_matrix;
Eigen::Matrix4d shank2foot1_matrix, shank2foot2_matrix, shank2foot3_matrix, shank2foot4_matrix;
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
    shank2foot1_matrix = findTransform("shank1", "foot1");
    shank2foot2_matrix = findTransform("shank2", "foot2");
    shank2foot3_matrix = findTransform("shank3", "foot3");
    shank2foot4_matrix = findTransform("shank4", "foot4");
    // 使用 matrix 进行进一步处理
    std::cout << "shank2foot2_matrix transform:\n"
              << shank2foot2_matrix << std::endl;
}

void tfstatic_Callback(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    for (const auto &tf : msg->transforms)
    {
        std::pair<std::string, std::string> key = {tf.header.frame_id, tf.child_frame_id};
        Eigen::Matrix4d transform_matrix = transformToEigenMatrix4d(tf.transform);
        // 更新或插入变换关系
        transformMap[key] = transform_matrix;
    }
    // shank2foot1_matrix = findTransform("shank1", "foot1");
    // shank2foot2_matrix = findTransform("shank2", "foot2");
    // shank2foot3_matrix = findTransform("shank3", "foot3");
    // shank2foot4_matrix = findTransform("shank4", "foot4");
    // 使用 matrix 进行进一步处理
    // std::cout << "shank2foot4_matrix transform:\n" << shank2foot4_matrix << std::endl;
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

void legIsMovingCallback(const std_msgs::Int32::ConstPtr &msg)
{
    ROS_INFO("Received leg_is_moving: %d", msg->data);
    switch (msg->data)
    {
    case BR_leg:
        // std::cout << "0\n" << std::endl;
        triangle_points = triangle_add_data(Odom2foot2_matrix, Odom2foot3_matrix, Odom2foot4_matrix);
        break;
    case FR_leg:
        // std::cout << "1\n" << std::endl;
        triangle_points = triangle_add_data(Odom2foot1_matrix, Odom2foot3_matrix, Odom2foot4_matrix);
        break;
    case FL_leg:
        // std::cout << "2\n" << std::endl;
        triangle_points = triangle_add_data(Odom2foot1_matrix, Odom2foot2_matrix, Odom2foot4_matrix);
        break;
    case BL_leg:
        // std::cout << "3\n" << std::endl;
        triangle_points = triangle_add_data(Odom2foot1_matrix, Odom2foot2_matrix, Odom2foot3_matrix);
        break;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "matrix_creator");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);
    // 创建发布者，发布到话题：/spibot_gazebo/draw/triangle_points
    // 注意：发布信息的话题首字母需要小写，否则rostopic没有看到话题
    triangle_pub = nh.advertise<std_msgs::Float32MultiArray>("/spibot_gazebo/draw/triangle_points", 50);
    // 订阅 /tf 话题
    ros::Subscriber robot_sub = nh.subscribe("/tf", 50, tfCallback);
    // 订阅 /tf_static 话题，即在urdf中关节定义为fixed的
    // ros::Subscriber robot_static_sub = nh.subscribe("/tf_static", 50, tfstatic_Callback);
    // 订阅 /spibot_gazebo/odometry 话题，并传递回调函数指针
    ros::Subscriber odom_sub = nh.subscribe("/spibot_gazebo/odometry", 50, odomCallback);
    // 订阅 /spibot_gazebo/states/leg_is_moving 话题,发布三个固定腿的位置
    ros::Subscriber leg_state_sub = nh.subscribe("/spibot_gazebo/states/leg_is_moving", 10, legIsMovingCallback);
    while (ros::ok())
    {
        // 可以在/spibot_gazebo/draw/triangle_points话题查看数据triangle_points
        triangle_pub.publish(triangle_points);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
