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

    bool swapXY = abs(y1 - y0) > abs(x1 - x0);// dy > dx
    if (swapXY) // dy > dx 在Y方向上迭代，swap变量　使其在X方向上迭代
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
    int error = 0;
    int y = y0;
    int pointX;
    int pointY;

    for (int x = x0; x <= x1; x ++ )
    {
        if (swapXY)
        {
            pointX = y;
            pointY = x;
        }
        else
        {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX)
        {
            y += ystep;
            error -= deltaX;
        }

        //不包含最后一个点
        if (pointX == x1 && pointY == y1) continue;

        //保存所有的点
        tmpIndex.SetIndex(pointX, pointY);

        gridIndexVector.push_back(tmpIndex);
    }
    return gridIndexVector;
}

/**
 * 从世界坐标系转换到栅格坐标系，主要是存在一个分辨率
 * resolution = 0.05, 世界坐标系，单位１在栅格坐标系中可以表示 1/resolution=20个栅格
 * 将机器人的实际位置，在900x900的栅格地图中找到对应的栅格，返回　GridIndex对象
*/
GridIndex ConverWorld2GridIndex(double x, double y)
{
    GridIndex index;
    //ceil()向上取整函数
    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

/**
 * 从栅格序号，转化到数组，因为栅格最终是按照顺序(width从小到大，height从低到高)依次存储到动态数组中的
*/
int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * mapParams.width;
}

/**
 * 判断index是否有效
 * 判断该栅格序号是否在设定栅格地图大小范围内
*/
bool isValidGridIndex(GridIndex index)
{
    if (index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;
    return false;
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

typedef struct
{
    std::vector<double> range_readings;
    std::vector<double> angle_readings;
}GeneralLaserScan;

/**
 * 占据栅格地图构建算法
 * 输入激光雷达数据和机器人位姿数据
 * 遍历所有帧，为pMap[]中的每个穿过的空间栅格或者击中栅格赋新值
 * 
*/
void OccupancyMapping()
{
    std::vector<GeneralLaserScan> scans;
    std::vector<Eigen::Vector3d> robotPoses;

    //遍历所有帧激光雷达数据
    for (int i = 0; i < scans.size(); i ++ )
    {
        //获取每一帧的激光雷达，机器人位姿数据
        GeneralLaserScan scan = scans[i];
        Eigen::Vector3d robotPose = robotPoses[i];

        //获取该帧机器人位姿的栅格序号
        GridIndex robotIndex = ConverWorld2GridIndex(robotPose(0), robotPose(1));

        //判断该帧机器人位姿栅格序号是否在设定的栅格地图范围内
        if (isValidGridIndex(robotIndex) == false) continue;

        //遍历该帧激光雷达数据的所有扫描点
        for (int id = 0; id < scan.range_readings.size(); id ++ )
        {
            //取出该激光雷达扫描点的距离和角度
            double dist = scan.range_readings[id];
            double angle = scan.angle_readings[id];
            //剔除异常数据，跳过该次，不做处理
            if (std::isinf(dist) || std::isnan(dist)) continue;
            //机器人航向角　机器人x轴与世界坐标系x轴夹角
            double theta = robotPose(2);

            //在旋转(与世界坐标系(像素坐标系下)平行)的激光雷达坐标系下的坐标x, y
            //激光雷达数据的角度逆时针变化
            //机器人航向角与世界坐标系x轴呈逆时针变化
            //这里的世界坐标系world_x,不是真实的世界坐标系，而是像素坐标系, y轴与真实的世界坐标系相反，这样是laser_y加负号的原因
            double laser_x = dist * cos(theta + angle);
            double laser_y = -dist * sin(theta + angle);

            //得到该激光，在世界坐标系下（像素坐标系下）的位置
            double world_x = laser_x + robotPose(0);
            double world_y = laser_y + robotPose(1);

            //将该激光扫描点在世界坐标系下的，转化为栅格序号
            GridIndex mapIndex = ConverWorld2GridIndex(world_x, world_y);

            //判断该激光扫描点的栅格，是否在自己设定的栅格地图900x900范围内，如果不在则跳过
            if (isValidGridIndex(mapIndex) == false) continue;

            //从机器人的栅格序号到该激光扫描点的栅格序号划线
            //找到两点之间途径的空闲栅格，将栅格序号存入std::vector<GridIndex>中
            std::vector<GridIndex> freeIndex = TraceLine(robotIndex.x, robotIndex.y, mapIndex.x, mapIndex.y);

            //遍历该扫描激光点通过的所有空闲栅格
            for (int k = 0; k < freeIndex.size(); k ++ )
            {
                GridIndex tmpIndex = freeIndex[k];
                //将空闲栅格的栅格，转化到数组，该数组用于存粗每一个栅格的数据
                int linearIndex = GridIndexToLinearIndex(tmpIndex);
                //取出栅格空闲规则
                int data = pMap[linearIndex];
                //根据栅格空闲规则，执行data += mapParams.log_free;
                if (data > 0) // 默认data = 50
                    data += mapParams.log_free; // log_free=-1, data将变小
                else
                    data = 0;
                //给该空闲栅格赋新值，最小为0;
                pMap[linearIndex] = data;
            }

            //更新该激光扫描点集中的栅格
            int tmpIndex = GridIndexToLinearIndex(mapIndex);
            int data = pMap[tmpIndex];
            //根据栅格击中规则，执行data += mapParams.log_occ;
            if (data < 100) //默认data=50
                data += mapParams.log_occ; //log_occ=2, data将变大
            else
                data = 100;
            //给击中的栅格赋新值，最大100
            pMap[tmpIndex] = data;
            //到这里，对一个位姿下的激光扫描数据经过的空闲栅格和击中栅格的pMap进行了重新赋值
        }
        //到这里，对一个位姿下的一帧激光扫描数据经过的空闲栅格和击中栅格的pMap进行了重新赋值
    }
    //到这里，对所有一帧激光扫描数据经过的空闲栅格和击中栅格的pMap进行了重新赋值
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
    // ros::Subscriber scanSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, ScanCallback);
    // ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>("odom", 1, OdomCallback);

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