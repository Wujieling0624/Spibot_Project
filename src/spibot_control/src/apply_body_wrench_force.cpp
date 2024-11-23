#include "ros/ros.h"
#include "gazebo_msgs/ApplyBodyWrench.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "apply_body_wrench_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    
    gazebo_msgs::ApplyBodyWrench srv;
    srv.request.body_name = "spibot::foot1";
    srv.request.reference_frame = "world";
    srv.request.reference_point.x = 0.0;
    srv.request.reference_point.y = 0.0;
    srv.request.reference_point.z = 0.0;
    srv.request.wrench.force.x = 0.0;
    srv.request.wrench.force.y = 0.0;
    srv.request.wrench.force.z = 10.0;
    srv.request.wrench.torque.x = 0.0;
    srv.request.wrench.torque.y = 0.0;
    srv.request.wrench.torque.z = 0.0;
    srv.request.start_time.sec = 0;
    srv.request.start_time.nsec = 0;
    srv.request.duration.sec = 10;
    srv.request.duration.nsec = 0;

    if (client.call(srv))
    {
        ROS_INFO("Service call successful");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}
