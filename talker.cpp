#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("drone_status", 10);

    ros::Rate rate(2);

    int count = 0;
    double altitude = 0.0;

    while (ros::ok())
    {
        if (altitude < 10.0)
            altitude += 0.5;

        std::stringstream ss;
        ss << "msg #" << count << ": altitude = " << altitude << " m";

        std_msgs::String msg;
        msg.data = ss.str();

        ROS_INFO("Publishing: %s", msg.data.c_str());
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
        ++count;
    }
    return 0;
}
