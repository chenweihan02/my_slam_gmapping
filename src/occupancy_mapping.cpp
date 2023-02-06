#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"

#include "occupancy_mapping.h"

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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "OccupancyMapping");

    ros::NodeHandle nh;
    ros::Publisher mapPub = nh.advertise<nav_msgs::OccupancyGrid>("laser_map", 1, true);

    std::vector<Eigen::Vector3d> robotPoses;


    //设置地图信息
    SetMapParams();
    //占用栅格地图构建算法
    
    //发布map，可视化
    PublishMap(mapPub);
}