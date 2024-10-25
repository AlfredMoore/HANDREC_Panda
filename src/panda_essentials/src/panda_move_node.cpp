#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pand_move_node");

    ros::NodeHandle nh;

    ros::Publisher dummy_publisher;
    dummy_publisher = nh.advertise<std_msgs::String>("/dummy", 1000);
    
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "an auto hello";
        msg.data = ss.str();

        ROS_INFO("%s", msg.data);

        dummy_publisher.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();


    }
    return 0;

}