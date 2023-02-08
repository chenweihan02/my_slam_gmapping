#ifndef GRIDSLAMPROCESSOR_H
#define GRIDSLAMPROCESSOR_H

#include <string>
#include <list>
#include <map>
#include <set>
#include <iomanip>
#include <climits>
#include <limits>
#include <fstream>
#include <vector>
#include <deque>
#include <omp.h>


#include <vector>
#include <deque>

// #include "../particlefilter/particlefilter.h"
#include "../utils/point.h"
#include "../sensor_range/rangereading.h"
#include "../scanmatcher/scanmatcher.h"
// #include "../motionmodel/motionmodel.h"


namespace GMapping {
    /**
     * 这个类实现了一个GridFastSLAM算法．实现了一个RBPF，每个粒子都拥有自己的地图和激光雷达位姿
     * 
     * 工作流程如下:
     *  每当收到里程计数据和激光雷达的数据之后，每个粒子的位姿根据运动模型来更新
     *  根据运动模型更新得到的新的位置随后被用来初始化scan-match算法
     *  scan-matcher为每个粒子执行了一个局部最优化算法
     *  scan-matcher被用运动模型得到的位置来初始化，然后根据自己的地图来优化位置
    */
   class GridSlamProcessor
   {
        public:
            /**
             * 树的节点，一个树储存了一整条轨迹，一个节点表示这条轨迹中的其中一个点　存储激光雷达的整条轨迹
             * 一个节点(TNode)包含了：
             *  该节点粒子的激光雷达位姿   pose
             *  指向父节点的指针        parent
             *  该节点激光雷达的读数    reading
            */
            struct TNode
            {
                TNode(const OrientedPoint& pose, TNode* parent=0);
                ~TNode();

                OrientedPoint pose;
                const RangeReading* reading;
                TNode* parent;
            };

            //用来定义一个节点数组，存储多条轨迹
            typedef std::vector<GridSlamProcessor::TNode*> TNodeVector;

            /**
             * 粒子滤波器中的粒子结构体
             * 每个粒子有自己的地图、位姿、权重、轨迹
             * 轨迹是按照时间顺序排列的，叶子节点表示最近的节点
            */

            struct Particle
            {
                //构造函数，初始化地图以及其他成员变量
                Particle(const ScanMatcherMap& map);
                //重载括号运算符
                inline operator double() const {return weight;}
                inline operator OrientedPoint() const{return pose;}
                //设置粒子权重
                inline void setWeight(double w) {weight = w;}
                //记录粒子当前地图
                ScanMatcherMap map;
                //记录粒子当前时刻激光雷达位姿
                OrientedPoint pose;
                //该粒子的当前权重
                double weight;
                //该粒子的累计权重
                double weightSum;
                //该粒子的节点，指向父节点
                TNode* node;
            };

            //重命名粒子元素的vector动态数组
            typedef std::vector<Particle> ParticleVector;

            //构造函数：初始化一些参数
            GridSlamProcessor();
            //销毁每一个粒子的轨迹树
            virtual ~GridSlamProcessor();
            //设置扫描匹配的参数
            void setMatchingParameters(double urange, double range, double sigma, int kernsize,
                                    double lopt, double aopt, int iterations, double likelihoodSigma=1,
                                    double likelihoodGain=1, unsigned int likelihoodSkip=0);
            //设置运动模型的参数
            void setMotionModelParameters(double srr,double srt, double str, double stt);
            //设置更新距离的参数
            void setUpdateDistances(double linear, double angular, double resampleThreshold);
            //设置更新频率
            void setUpdatePeriod(double p) {period_ = p;}

            /**
             * 设置地图尺寸，分辨率，初始位姿，清空每个粒子
             * new一个轨迹树的根节点，初始化，初始位姿
             * 定义一个地图，为size个粒子，初始化，地图、位姿、权重、轨迹
            */
            void init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta,
                    OrientedPoint initialPose=OrientedPoint(0, 0, 0));
            //SLAM处理程序
            bool processScan(const RangeReading& reading, int adaptParticles=0);
            //获得粒子的数量
            inline const ParticleVector& getParticles() const {return m_particles;}
            //获得最优位姿粒子的序号
            int getBestParticleIndex() const;
            //扫描匹配对象
            ScanMatcher m_matcher;
            
            //定义大量的成员的set和get函数，成员本身有get和set函数
            //这些成员在自己的类中，已经有了自己的get和set函数
            MEMBER_PARAM_SET_GET(m_matcher, double,       laserMaxRange,          protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, double,       usableRange,            protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, double,       gaussianSigma,          protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, double,       likelihoodSigma,        protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, int,          kernelSize,             protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, double,       optAngularDelta,        protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, double,       optLinearDelta,         protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, unsigned int, optRecursiveIterations, protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, unsigned int, likelihoodSkip,         protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, bool,         generateMap,            protected, public, public);
            MEMBER_PARAM_SET_GET(m_matcher, bool,         enlargeStep,            protected, public, public);

            //定义大量的成员的set和get函数，成员本身没有有get和set函数，直接读取
            STRUCT_PARAM_SET_GET(m_motionModel, double, srr, protected, public, public);
            STRUCT_PARAM_SET_GET(m_motionModel, double, srt, protected, public, public);
            STRUCT_PARAM_SET_GET(m_motionModel, double, str, protected, public, public);
            STRUCT_PARAM_SET_GET(m_motionModel, double, stt, protected, public, public);

            //认为匹配成功的最小得分阈值
            PARAM_SET_GET(double, minimumScore,              protected, public, public);
        protected:
            //记录上一次更新时间
            double                      last_update_time;
            //两次更新时间的最小间隔
            double                      period_;
            //激光束的数量
            unsigned int                m_beams;
            //粒子数组
            ParticleVector              m_particles;
            //重采样之后，剩余的粒子的下标，这个下标的是会重复的
            std::vector<unsigned int>   m_indexes;
            //所有粒子的当前的权重
            std::vector<double>         m_weights;
            //粒子运动模型
            MotionModel                 m_motionModel;
            //上一次的位姿
            OrientedPoint               m_odoPose;
            //激光雷达的位姿
            OrientedPoint               m_pose;
            //被处理过的激光雷达帧数
            int                         m_count;
            //记录两帧之间激光雷达直线位移
            double                      m_linearDistance;
            //记录两帧之间激光雷达角度位移
            double                      m_angularDistance;
            //定义一大堆变量以及其set和get函数
            //地图尺寸，精度
            PARAM_GET(double, xmin, protected, public);
            PARAM_GET(double, ymin, protected, public);
            PARAM_GET(double, xmax, protected, public);
            PARAM_GET(double, ymax, protected, public);
            PARAM_GET(double, delta, protected, public);
            //记录粒子的离散程度，论文中有计算公式
            PARAM_GET(double, neff, protected, public);
            //两帧之间激光雷达直线、角度位移阈值
            PARAM_SET_GET(double, linearThresholdDistance,  protected, public, public);
            PARAM_SET_GET(double, angularThresholdDistance, protected, public, public);
            //
            PARAM_SET_GET(double, obsSigmaGain,             protected, public, public);
            //粒子选择性重采样的阈值
            PARAM_SET_GET(double, resampleThreshold,        protected, public, public);
   };       
};

#endif