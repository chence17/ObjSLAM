/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-12 19:36:14
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2,
 * 这个头文件定义了ORB-SLAM2主线程(系统)的结构,
 * 其他的各个模块都是从这里被调用的.
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

namespace ORB_SLAM2 {
// 需要使用到的其他模块的前置声明.
class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

// ORB-SLAM2主线程(系统).
class System {
 public:
  // enum 相机传感器类型.
  // [0]MONOCULAR: 单目传感器
  // [1]STEREO: 双目传感器
  // [2]RGBD: RGBD传感器
  enum eSensor { MONOCULAR = 0, STEREO = 1, RGBD = 2 };

 public:
  /**
   * @brief ORB-SLAM2主线程(系统)的构造函数.
   * @note 初始化ORB-SLAM2主线程(系统)系统.
   * @param strVocFile: ORB字典文件的路径
   * @param strSettingsFile: 配置文件的路径
   * @param sensor: 使用的传感器类型
   * @param bUseViewer: 是否使用可视化界面 #TODO
   */
  System(const string &strVocFile, const string &strSettingsFile,
         const eSensor sensor, const bool bUseViewer = true);

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
  cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
                      const double &timestamp);

  /**
   * @brief RGBD相机追踪函数.
   * @note 处理输入的RGBD相机帧, RGB图像和深度图像必须经过配准.
   * @param im: RGB图像, 彩色图像(CV_8UC3)或灰度图像(CV8U).
   * @param depthmap: 深度图像(CV_32F).
   * @param timestamp: 时间戳.
   * @return cv::Mat 输入帧的相机位姿, 追踪失败则返回NULL.
   */
  cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap,
                    const double &timestamp);

  /**
   * @brief 单目相机追踪函数.
   * @note 处理输入的单目相机帧.
   * @param im: 图像, 彩色图像(CV_8UC3)或灰度图像(CV8U).
   * @param timestamp: 时间戳.
   * @return cv::Mat 输入帧的相机位姿, 追踪失败则返回NULL.
   */
  cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

  /**==============================================
   * *                   模式切换函数
   *   下面是系统工作模式的切换函数.
   *   // NOTE: 毕设相关, 模式切换相关
   *
   *=============================================**/

  /**
   * @brief 激活纯定位模式.
   * @note 激活纯定位模式, 在纯定位模式下只有运动追踪模块进行工作,
   * 局部建图模块停止工作.
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
   * @note 所有的线程将会关闭, 请在保存完相关数据后执行此函数,
   * 执行此函数后将丢失所有数据.
   * @return None
   */
  void Shutdown();

  /**==============================================
   * *                   轨迹保存函数
   *   下面是系统的数据保存函数.
   *
   *=============================================**/

  /**
   * @brief 相机轨迹保存函数( TUM 格式) #NOTE: 仅支持双目和 RGBD 相机,
   * 不支持单目相机.
   * @note 以 TUM
   * 格式保存相机的运动轨迹, 这个函数将会在系统关闭函数函数中被首先调用, TUM
   * 格式相关细节请参考 http://vision.in.tum.de/data/datasets/rgbd-dataset.
   * @param filename: 相机轨迹保存文件名
   * @return None
   */
  void SaveTrajectoryTUM(const string &filename);

  /**
   * @brief 关键帧轨迹保存函数( TUM 格式) #NOTE: 支持所有相机.
   * @note 以 TUM
   * 格式保存关键帧轨迹, 这个函数将会在系统关闭函数函数中被首先调用, TUM
   * 格式相关细节请参考 http://vision.in.tum.de/data/datasets/rgbd-dataset.
   * #TODO
   * 是不是这也意味着, 我可以使用g2o_viewer这样的软件去查看并且进行优化实验？
   * @param filename: 关键帧轨迹保存文件名
   * @return None
   */
  void SaveKeyFrameTrajectoryTUM(const string &filename);

  /**
   * @brief 相机轨迹保存函数( KITTI 格式) #NOTE: 仅支持双目和 RGBD 相机,
   * 不支持单目相机.
   * @note 以 KITTI
   * 格式保存相机的运动轨迹, 这个函数将会在系统关闭函数函数中被首先调用, KITTI
   * 格式相关细节请参考 http://www.cvlibs.net/datasets/kitti/eval_odometry.php.
   * @param filename: 相机轨迹保存文件名
   * @return None
   */
  void SaveTrajectoryKITTI(const string &filename);

  // NOTE: 毕设相关, 查看 单目 不支持相机轨迹保存的原因,
  // 考虑获取纯VO相关数据时使用的相机类型.

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
  //获取最近的运动追踪状态、地图点追踪状态、特征点追踪状态()
  /**
   * @brief 获取追踪模块当前状态.
   * @note 可以在相机追踪函数运行结束后使用.
   * @return int 追踪模块状态, 详见 Tracking::eTrackingState.
   */
  int GetTrackingState();

  /**
   * @brief 获取追踪模块对当前帧提取的关键点对应的地图点的向量.
   * @note 可以在相机追踪函数运行结束后使用.
   * @return std::vector<MapPoint *> 当前帧关键点对应的地图点所构成的向量.
   */
  std::vector<MapPoint *> GetTrackedMapPoints();

  /**
   * @brief 获取追踪模块对当前帧提取的校正后的关键点的向量.
   * @note 可以在相机追踪函数运行结束后使用.
   * @return std::vector<cv::KeyPoint> 当前帧校正后的关键点所构成的向量.
   */
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

 private:
  // (System::eSensor) System 所使用的相机传感器类型.
  eSensor mSensor;

  // (ORBVocabulary *) 指向 System 所使用的 ORB 字典的指针.
  // ORB 字典是用来进行位置识别和特征匹配的.
  ORBVocabulary *mpVocabulary;

  // (KeyFrameDatabase *) 指向 System 所使用的关键帧数据库的指针.
  // 关键帧数据库是用来进行重定位和回环检测的.
  KeyFrameDatabase *mpKeyFrameDatabase;

  // (Map *) 指向 System 所使用的地图的指针.
  // 地图储存所有关键帧和地图点的指针.
  Map *mpMap;

  // (Tracking *) 指向 System 所使用的追踪模块的指针.
  // 追踪模块主要实现以下功能:
  // 1. 运动追踪;
  // 2. 插入新关键帧;
  // 3. 加入新地图点;
  // 4. 重定位.
  Tracking *mpTracker;

  // (LocalMapping *) 指向 System 所使用的局部建图模块的指针.
  // 局部建图模块主要实现以下功能:
  // 1. 管理局部地图;
  // 2. 进行局部联合优化(BA, Bundle Adjustment).
  LocalMapping *mpLocalMapper;

  // (LoopClosing *) 指向 System 所使用的回环检测模块的指针.
  // 回环检测模块主要实现一下功能:
  // 1. 在所有的关键帧中搜索回环;
  // 2. 在检测到回环时进行位姿图优化; #NOTE: 位姿图优化是不是也开了个新新线程呢?
  // 3. 在检测到回环时开启新线程进行全局联合优化(BA, Bundle Adjustment).
  LoopClosing *mpLoopCloser;

  // (Viewer *) 指向 System 所使用的显示模块的指针.
  // 显示模块主要实现以下功能:
  // 1. 绘制地图;
  // 2. 显示当前相机位姿.
  // 注: 显示模块依赖 Pangolin 包.
  Viewer *mpViewer;

  // (FrameDrawer *) 指向 System 所使用的帧绘制模块的指针.
  // 帧绘制模块主要实现以下功能:
  // #NOTE: 这个模块到底是干啥的?
  FrameDrawer *mpFrameDrawer;

  // (MapDrawer *) 指向 System 所使用的地图绘制模块的指针.
  // 地图绘制模块主要实现以下功能:
  // #NOTE: 这个模块到底是干啥的?
  MapDrawer *mpMapDrawer;

  // System threads: Local Mapping, Loop Closing, Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the
  // System object.
  //系统除了在主进程中进行运动追踪工作外,
  //会创建局部建图线程、回环检测线程和查看器线程.

  /**==============================================
   * *             System 各线程运行关系
   *   追踪线程存在于主程序线程中.
   *   建图, 回环检测和显示线程是由主程序线程创建和管理的.
   *   #NOTE: 是否可以改成追踪线程也是独立的?
   *   #NOTE: 主程序线程仅仅只是控制?
   *
   *=============================================**/

  // (std::thread *) 指向 System 局部建图线程的指针.
  // 局部建图线程主要进行以下操作:
  // #NOTE: 这个线程到底是干啥的?
  std::thread *mptLocalMapping;

  // (std::thread *) 指向 System 回环检测线程的指针.
  // 回环检测线程主要进行以下操作:
  // #NOTE: 这个线程到底是干啥的?
  std::thread *mptLoopClosing;

  // (std::thread *) 指向 System 显示线程的指针.
  // 显示线程主要进行以下操作:
  // #NOTE: 这个线程到底是干啥的?
  std::thread *mptViewer;

  // (std::mutex) 系统复位相关. #TODO: 完善注释
  // #NOTE: 不清楚这里的 mMutexReset 是干嘛的?
  // #NOTE: 为什么要定义 std::mutex 数据类型呢?
  std::mutex mMutexReset;

  // (bool) System 的复位标志变量.
  // true 代表进行系统复位操作;
  // false 代表不进行系统复位操作.
  // #NOTE: System 的复位流程是怎样的呢?
  bool mbReset;

  // (std::mutex) 系统模式改变相关. #TODO: 完善注释
  // #NOTE: 不清楚这里的 mMutexMode 是干嘛的?
  // #NOTE: 为什么要定义 std::mutex 数据类型呢?
  std::mutex mMutexMode;

  // (bool) System 激活纯定位模式的标志变量
  // true 代表进行激活纯定位模式的操作;
  // false 代表不进行激活纯定位模式的操作.
  // NOTE: 纯定位模式又称纯视觉里程计模式或纯 VO 模式.
  bool mbActivateLocalizationMode;

  // (bool) System 反激活纯定位模式的标志变量
  // true 代表进行反激活纯定位模式的操作;
  // false 代表不进行反激活纯定位模式的操作.
  // NOTE: 反激活的含义是指从纯定位模式中切换到正常模式.
  bool mbDeactivateLocalizationMode;

  // Tracking state
  // 追踪状态标志, 注意前三个的类型和上面的函数类型相互对应
  // (int) System 追踪模块的状态变量, 详见 Tracking::eTrackingState.
  int mTrackingState;

  // (std::vector\<MapPoint *>) System 追踪模块追踪到的地图点向量.
  // 1. 追踪模块在当前输入帧中提取关键点;
  // 2. 将提取到的关键点转换为对应的地图点;
  // 3. 由这些地图点构成地图点向量.
  std::vector<MapPoint *> mTrackedMapPoints;

  // (std::vector\<cv::KeyPoint>) System 追踪模块追踪到关键点矫正结果向量.
  // 1. 追踪模块在当前输入帧中提取关键点;
  // 2. 对提取到的关键点进行校正;
  // 3. 由这些关键点的矫正结果构成关键点矫正结果向量.
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;

  // (std::mutex) 追踪模块状态相关. #TODO: 完善注释
  // #NOTE: 不清楚这里的 mMutexState 是干嘛的?
  // #NOTE: 为什么要定义 std::mutex 数据类型呢?
  std::mutex mMutexState;
};

}  // namespace ORB_SLAM2

#endif  // INC_SYSTEM_H_
