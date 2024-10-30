#include <sstream>

#include <ros/init.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_move_node");
    ros::NodeHandle node_handle("~");

    actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

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