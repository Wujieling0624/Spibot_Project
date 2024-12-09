#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
/*
任务：监听/spibot_gazebo/odometry话题，并发布到rviz的tf树中，确保rviz能订阅到这个话题
*/
// first, we'll publish the transform over tf
geometry_msgs::TransformStamped odom_trans;

// 回调函数，当接收到/dometry话题的消息时被调用
void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;
    odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
    odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
    odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
    odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    ros::Rate r(50);
    ros::Subscriber sub = nh.subscribe("/spibot_gazebo/odometry", 10, odometryCallback);
    while (nh.ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "dummy";
        odom_broadcaster.sendTransform(odom_trans);
        ROS_INFO("odom msg send.");
        r.sleep();
    }
    return 0;
}
