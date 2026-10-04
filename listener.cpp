#include <ros/ros.h>
#include <std_msgs/String.h>

int received = 0;

void statusCallback(const std_msgs::String::ConstPtr &msg)
{
    ++received;
    ROS_INFO("Received #%d: [%s]", received, msg->data.c_str());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("drone_status", 10, statusCallback);

    ros::spin();
    return 0;
}
