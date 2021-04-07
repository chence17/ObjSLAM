/**
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-06 18:54:52
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2, 这个头文件定义了ORB-SLAM2主线程(系统)的结构, 其他的各个模块都是从这里被调用的.
 * @FilePath: /Localization/inc/System.h
 */

#ifndef INC_SYSTEM_H_
#define INC_SYSTEM_H_

// 公用库.
#include <opencv2/core/core.hpp>
#include <string>
#include <thread>

// ORB-SLAM2中的其他模块.
#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "Viewer.h"

/**==============================================
 * *                   变量命名规则
 *   类的公有成员变量(public)的名称带有前缀m.
 *   类的私有成员变量(private)的名称前缀为空.
 *   枚举变量(enum)的名称带有前缀e.
 *   指针变量(pointer)的名称带有前缀p.
 *   线程变量(thread)的名称带有前缀t.
 *
 *   前缀先后顺序: m>p>t
 *
 *=============================================**/

namespace ORB_SLAM2
{
    // 需要使用到的其他模块的前置声明.
    class Viewer;
    class FrameDrawer;
    class Map;
    class Tracking;
    class LocalMapping;
    class LoopClosing;

    // ORB-SLAM2主线程(系统).
    class System
    {
    public:
        // enum 表示ORB-SLAM2主线程(系统)所使用的传感器类型.
        // [0]MONOCULAR: 单目传感器
        // [1]STEREO: 双目传感器
        // [2]RGBD: RGBD传感器
        enum eSensor
        {
            MONOCULAR = 0,
            STEREO = 1,
            RGBD = 2
        };

    public:
        /**
         * @brief ORB-SLAM2主线程(系统)的构造函数.
         * @note 初始化ORB-SLAM2主线程(系统)系统.
         * @param strVocFile: ORB字典文件的路径
         * @param strSettingsFile: 配置文件的路径
         * @param sensor: 使用的传感器类型
         * @param bUseViewer: 是否使用可视化界面 #TODO
         */
        System(const string &strVocFile,
               const string &strSettingsFile,
               const eSensor sensor,
               const bool bUseViewer = true);

        /**==============================================
        * *                   相机追踪函数
        *   下面是针对三种不同类型的传感器所设计的三种相机追踪函数.
        *   输入图像可以是彩色图像(CV_8UC3)也可以是灰度图像(CV8U).
        *   如果输入是彩色图像则会自动转换为灰度图像.
        *   函数返回估计的相机位姿(cv::Mat), 追踪失败则返回NULL.
        *
        *=============================================**/

        /**
         * @brief 双目相机追踪函数.
         * @note 处理输入的双目相机帧, 左右目的图片必须经过同步和校正.
         * @param imLeft: 左目图像, 彩色图像(CV_8UC3)或灰度图像(CV8U).
         * @param imRight: 右目图像, 彩色图像(CV_8UC3)或灰度图像(CV8U).
         * @param timestamp: 时间戳.
         * @return cv::Mat 输入帧的相机位姿, 追踪失败则返回NULL.
         */
        cv::Mat TrackStereo(const cv::Mat &imLeft,
                            const cv::Mat &imRight,
                            const double &timestamp);

        /**
         * @brief RGBD相机追踪函数.
         * @note 处理输入的RGBD相机帧, RGB图像和深度图像必须经过配准.
         * @param im: RGB图像, 彩色图像(CV_8UC3)或灰度图像(CV8U).
         * @param depthmap: 深度图像(CV_32F).
         * @param timestamp: 时间戳.
         * @return cv::Mat 输入帧的相机位姿, 追踪失败则返回NULL.
         */
        cv::Mat TrackRGBD(const cv::Mat &im,
                          const cv::Mat &depthmap,
                          const double &timestamp);

        /**
         * @brief 单目相机追踪函数.
         * @note 处理输入的单目相机帧.
         * @param im: 图像, 彩色图像(CV_8UC3)或灰度图像(CV8U).
         * @param timestamp: 时间戳.
         * @return cv::Mat 输入帧的相机位姿, 追踪失败则返回NULL.
         */
        cv::Mat TrackMonocular(const cv::Mat &im,
                               const double &timestamp);

        /**==============================================
        * *                   模式切换函数
        *   下面是系统工作模式的切换函数.
        *   // NOTE: 毕设相关, 模式切换相关
        *
        *=============================================**/

        /**
         * @brief 激活纯定位模式.
         * @note 激活纯定位模式, 在纯定位模式下只有运动追踪模块进行工作, 局部建图模块停止工作.
         * @return None
         */
        void ActivateLocalizationMode();

        /**
         * @brief 禁用纯定位模式.
         * @note 禁用纯定位模式, 局部建图模块恢复工作.
         * @return None
         */
        void DeactivateLocalizationMode();

        /**==============================================
         * *                   系统监测函数
         *   下面是检测系统相关变化的函数.
         *
         *=============================================**/

        /**
         * @brief 地图变化监测函数
         * @note 获取自从上次调用本函数后, 地图是否发生了比较大的变化.
         * @return bool 地图发生了比较大的变化返回 true, 否则返回 false.
         */
        bool MapChanged();

        /**==============================================
         * *                   系统控制函数
         *    下面是系统的控制函数.
         *
         *=============================================**/

        /**
         * @brief 系统复位函数
         * @note 复位系统
         * @return None
         */
        void Reset();

        /**
         * @brief 系统关闭函数
         * @note 所有的线程将会关闭, 请在保存完相关数据后执行此函数, 执行此函数后将丢失所有数据.
         * @return None
         */
        void Shutdown();

        /**==============================================
        * *                   轨迹保存函数
        *   下面是系统的数据保存函数.
        *
        *=============================================**/

        /**
         * @brief 相机轨迹保存函数( TUM 格式) #NOTE: 仅支持双目和 RGBD 相机, 不支持单目相机.
         * @note 以 TUM 格式保存相机的运动轨迹，这个函数将会在系统关闭函数函数中被首先调用, TUM 格式相关细节请参考 http://vision.in.tum.de/data/datasets/rgbd-dataset.
         * @param filename: 相机轨迹保存文件名
         * @return None
         */
        void SaveTrajectoryTUM(const string &filename);

        /**
         * @brief 关键帧轨迹保存函数( TUM 格式) #NOTE: 支持所有相机.
         * @note 以 TUM 格式保存关键帧轨迹，这个函数将会在系统关闭函数函数中被首先调用, TUM 格式相关细节请参考 http://vision.in.tum.de/data/datasets/rgbd-dataset. #TODO 是不是这也意味着，我可以使用g2o_viewer这样的软件去查看并且进行优化实验？
         * @param filename: 关键帧轨迹保存文件名
         * @return None
         */
        void SaveKeyFrameTrajectoryTUM(const string &filename);

        /**
         * @brief 相机轨迹保存函数( KITTI 格式) #NOTE: 仅支持双目和 RGBD 相机, 不支持单目相机.
         * @note 以 KITTI 格式保存相机的运动轨迹，这个函数将会在系统关闭函数函数中被首先调用, KITTI 格式相关细节请参考 http://www.cvlibs.net/datasets/kitti/eval_odometry.php.
         * @param filename: 相机轨迹保存文件名
         * @return None
         */
        void SaveTrajectoryKITTI(const string &filename);

        // NOTE: 毕设相关, 查看 单目 不支持相机轨迹保存的原因, 考虑获取纯VO相关数据时使用的相机类型.

        /**==============================================
         * *                   地图数据操作
         *   以下是与地图数据相关的操作函数.
         *   #TODO: 地图加载和保存函数暂未实现, 可以自行实现.
         *
         *=============================================**/

        // SaveMap(const string &filename);
        // LoadMap(const string &filename);

        /**==============================================
        * *                   状态获取函数
        *   一下是最近处理的几帧的相关状态的获取函数.
        *   可以在相机追踪函数调用后使用一下函数.
        *
        *=============================================**/

        // Information from most recent processed frame
        // You can call this right after TrackMonocular (or stereo or RGBD)
        //获取最近的运动追踪状态、地图点追踪状态、特征点追踪状态（）
        /**
         * @brief 获取追踪线程当前状态.
         * @note 可以在相机追踪函数运行结束后使用.
         * @return int 追踪线程状态, 详见 Tracking::eTrackingState.
         */
        int GetTrackingState();

        /**
         * @brief 获取追踪线程对当前帧提取的关键点对应的地图点的向量.
         * @note 可以在相机追踪函数运行结束后使用.
         * @return std::vector<MapPoint *> 当前帧关键点对应的地图点所构成的向量.
         */
        std::vector<MapPoint *> GetTrackedMapPoints();

        /**
         * @brief 获取追踪线程对当前帧提取的校正后的关键点的向量.
         * @note 可以在相机追踪函数运行结束后使用.
         * @return std::vector<cv::KeyPoint> 当前帧校正后的关键点所构成的向量.
         */
        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    private:
        // Input sensor
        // 传感器类型
        eSensor mSensor;

        // ORB vocabulary used for place recognition and feature matching.
        // 一个指针指向ORB字典
        ORBVocabulary *mpVocabulary;

        // KeyFrame database for place recognition (relocalization and loop detection).
        // 关键帧数据库的指针，这个数据库用于重定位和回环检测
        KeyFrameDatabase *mpKeyFrameDatabase;

        //指向地图（数据库）的指针
        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map *mpMap;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        // 追踪器，除了进行运动追踪外还要负责创建关键帧、创建新地图点和进行重定位的工作。详细信息还得看相关文件
        Tracking *mpTracker;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
        //局部建图器。局部BA由它进行。
        LocalMapping *mpLocalMapper;

        // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
        // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
        // 回环检测器，它会执行位姿图优化并且开一个新的线程进行全局BA
        LoopClosing *mpLoopCloser;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        // 查看器，可视化 界面
        Viewer *mpViewer;

        //帧绘制器
        FrameDrawer *mpFrameDrawer;
        //地图绘制器
        MapDrawer *mpMapDrawer;

        // System threads: Local Mapping, Loop Closing, Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        //系统除了在主进程中进行运动追踪工作外，会创建局部建图线程、回环检测线程和查看器线程。
        std::thread *mptLocalMapping;
        std::thread *mptLoopClosing;
        std::thread *mptViewer;

        // Reset flag
        //复位标志，注意这里目前还不清楚为什么要定义为std::mutex类型 TODO
        std::mutex mMutexReset;
        bool mbReset;

        // Change mode flags
        //模式改变标志
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Tracking state
        // 追踪状态标志，注意前三个的类型和上面的函数类型相互对应
        int mTrackingState;
        std::vector<MapPoint *> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
        std::mutex mMutexState;
    };

} // namespace ORB_SLAM

#endif // INC_SYSTEM_H_
