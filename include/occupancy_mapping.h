#ifndef OCCUPANCY_MAPPING_H
#define OCCUPANCY_MAPPING_H

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <eigen3/Eigen/Core>

/**
 * 自定义名为GridIndex（栅格序号）的数据类型
 * 每一个GridIndex, 包含x, y坐标
*/
typedef struct
{
    int x;
    int y;

    void SetIndex(int x_, int y_)
    {
        x = x_;
        y = y_;
    }
}GridIndex;



/**
 * 自定义名为MapParams（地图参数）的数据类型
 * 包含地图的基本信息：分辨率　地图原点　地图大小　偏移等
 * 
*/
typedef struct
{
    double log_occ, log_free;
    double resolution;
    double origin_x, origin_y;
    int width, height;
    int offset_x, offset_y;
}MapParams;

MapParams mapParams;
//地图指针
unsigned char* pMap;

#endif