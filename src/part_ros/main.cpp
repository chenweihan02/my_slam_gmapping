#include "ros/ros.h"
#include "my_slam_gmapping.h"

/**
 * 基于滤波器的SLAM算法-《gmapping算法的删减版》+《加入激光雷达运动畸变去除》
 * 1.删除几乎所有不需要的代码，对代码的运行结构也进行了调整
 * 2.对该代码进行详细中文注释，以及对核心代码进行更改
 * 3.将激光雷达运动畸变去除算法，直接加入删减版的gmapping算法中，算法文件在part_data文件夹
 * 2023/2/8
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_slam_gmapping");

    MySlamGmapping slamer;
    slamer.startLiveSlam();
    ros::spin();

    return 0;
}