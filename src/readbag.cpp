/**
 * 读取bag 文件
*/
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_read");
    ros::NodeHandle nh;

    ROS_INFO("read file =========================");
    return 0;
}
