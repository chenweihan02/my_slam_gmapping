#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "occupancy_mapping.h"
#include "nav_msgs/Odometry.h"

/**
 * Bresenham 2D画线算法　来进行计算两点之间的　grid cell
 * 找到两点之间的空闲，将栅格序号存入 std::vector<GridIndex>中
*/
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
    GridIndex tmpIndex;
    std::vector<GridIndex> gridIndexVector;

    bool steep = abs(y1 - y0) > abs(x1 - x0);// dy > dx
    if (steep) // dy > dx 在Y方向上迭代，swap变量　使其在X方向上迭代
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    //======================================================
    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int ystep = (y1 > y0) ? 1 : -1; // y的步长
    
    for (int x = x0; x <= x1; x ++ )
    {
        
    }

}


//设置地图参数
void SetMapParams()
{
    //地图大小　分辨率
    mapParams.width = 900;
    mapParams.height = 900;
    mapParams.resolution = 0.05; // 0.05m 1/0.05=20个栅格
    //假设free=-1, occ=2
    mapParams.log_free = -1;
    mapParams.log_occ = 2;
    //
    mapParams.origin_x = 0;
    mapParams.origin_y = 0;
    //地图的原点，即是机器人默认位置
    mapParams.offset_x = 100;
    mapParams.offset_y = 100;
    //为地图指针申请空间
    pMap = new unsigned char[mapParams.width * mapParams.height];

    //每一个栅格代表的值，初始化为50
    for (int i = 0; i < mapParams.width * mapParams.height; i ++ )
        pMap[i] = 50;
}

void PublishMap(ros::Publisher& map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 0.0;

    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    for (int i = 0; i < mapParams.width * mapParams.height; i ++ )
    {
        if (pMap[i] == 50)  //未知栅格
        {
            rosMap.data[i] = -1.0;
        }
        else if (pMap[i] < 50)  //空闲栅格
        {
            // rosMap.data[i] = 0;      //gmapping方式
            rosMap.data[i] = pMap[i];   //cartographer方式 TODO
        }
        else if (pMap[i] > 50)  //击中栅格
        {
            // rosMap.data[i] = 100;
            rosMap.data[i] = pMap[i];
        }
    }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}

void DestoryMap()
{
    if (pMap != NULL)
        delete pMap;
}

std::vector<Eigen::Vector3d> robotPoses;


/**
 * 占据栅格地图构建算法
 * 输入激光雷达数据和机器人位姿数据
 * 遍历所有帧，为pMap[]中的每个穿过的空间栅格或者击中栅格赋新值
 * 
*/
void OccupancyMapping()
{
    
}


void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{

}

void OdomCallback()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "OccupancyMapping");

    ros::NodeHandle nh;
    ros::Publisher mapPub = nh.advertise<nav_msgs::OccupancyGrid>("laser_map", 1, true);
    ros::Subscriber scanSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, ScanCallback);
    ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>("odom", 1, OdomCallback);


    //设置地图信息
    SetMapParams();
    //占用栅格地图构建算法
    OccupancyMapping();
    //发布map，可视化
    PublishMap(mapPub);
    //ROS消息回调函数
    ros::spin();
    //销毁地图
    DestoryMap();

    std::cout << "Release Memory!" << std::endl;
}