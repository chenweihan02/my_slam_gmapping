#ifndef __LIDAR_UNDISTORTION_H__
#define __LIDAR_UNDISTORTION_H__

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <string>

/**
 * 2D激光雷达运动畸变去除
*/

//雷达运动畸变去除类
class LidarMotionCalibrator
{
    public:
        //构造函数
        LidarMotionCalibrator(std::string scan_frame_name, std::string odom_name);
        //析构函数，释放tf_
        ~LidarMotionCalibrator();
        //激光雷达运动畸变去除函数
        void lidarCalibration(std::vector<double>& ranges, std::vector<double>& angles,
                            ros::Time startTime, ros::Time endTime,
                            tf::TransformListener* tf_);
        //从tf缓存数据中，寻找对应时间戳的里程计位姿
        bool getLaserPose(tf::Stamped<tf::Pose>& odom_pose, ros::Time dt, tf::TransformListener* tf_);
        //根据传入参数，对任意一个分段进行插值
        void lidarMotionCalibration(tf::Stamped<tf::Pose> frame_base_pose, 
                                    tf::Stamped<tf::Pose> frame_start_pose,
                                    tf::Stamped<tf::Pose> frame_end_pose,
                                    std::vector<double>& ranges,
                                    std::vector<double>& angles,
                                    int startIndex, int& beam_number);
    public:
        //针对各自的清空需要更改的名字，自行更改
        std::string scan_frame_name_;
        std::string odom_name_;
};

#endif