/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-12 22:37:43
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/Tracking.h
 */

#ifndef INC_TRACKING_H_
#define INC_TRACKING_H_

// 公用库.
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

// ORB-SLAM2中的其他模块.
#include "Frame.h"
#include "FrameDrawer.h"
#include "Initializer.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "System.h"
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
class LocalMapping;
class LoopClosing;
class System;

// 追踪线程数据类型.
// 实现追踪当前帧的功能.
class Tracking {
 public:
  /**
   * @brief 追踪线程数据类型的构造函数.
   * @note note
   * @param pSys: 系统实例
   * @param pVoc: 字典指针
   * @param pFrameDrawer: 帧绘制器
   * @param pMapDrawer: 地图绘制器
   * @param pMap: 地图句柄
   * @param pKFDB: 关键帧数据库句柄
   * @param strSettingPath: 配置文件路径
   * @param sensor: 传感器类型
   */
  Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
           MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB,
           const string &strSettingPath, const int sensor);

  /**==============================================
   * *                   输入处理函数
   *   Preprocess the input and call Track().
   *   Extract features and performs
   *   stereo matching.
   *   下面的函数都是对不同的传感器输入的图像进行处理
   *   (转换成为灰度图像),并且调用Tracking线程
   *
   *=============================================**/

  /**
   * @brief 处理双目输入
   * @note
   * @param imRectLeft: 左目图像
   * @param imRectRight: 右目图像
   * @param timestamp: 时间戳
   * @return (cv::Mat) 世界坐标系到该帧相机坐标系的变换矩阵
   */
  cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight,
                          const double &timestamp);

  /**
   * @brief 处理RGBD输入的图像
   * @note
   * @param imRGB: 彩色图像
   * @param imD: 深度图像
   * @param timestamp: 时间戳
   * @return (cv::Mat) 世界坐标系到该帧相机坐标系的变换矩阵
   */
  cv::Mat GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD,
                        const double &timestamp);

  /**
   * @brief 处理单目输入图像
   * @note
   * @param im: 图像
   * @param timestamp: 时间戳
   * @return (cv::Mat) 世界坐标系到该帧相机坐标系的变换矩阵
   */
  cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

  /**
   * @brief 设置局部地图句柄
   * @note
   * @param pLocalMapper: 局部建图器
   * @return None
   */
  void SetLocalMapper(LocalMapping *pLocalMapper);

  /**
   * @brief 设置回环检测器句柄
   * @note
   * @param pLoopClosing: 回环检测器
   * @return None
   */
  void SetLoopClosing(LoopClosing *pLoopClosing);

  /**
   * @brief 设置可视化查看器句柄
   * @note
   * @param pViewer: 可视化查看器
   * @return None
   */
  void SetViewer(Viewer *pViewer);

  /**
   * @brief Load new settings
   * @note
   * The focal length should be similar or scale prediction will fail when
   * projecting points
   * #TODO: Modify MapPoint::PredictScale to take into account focal lenght
   * @param strSettingPath: 配置文件路径
   * @return None
   */
  void ChangeCalibration(const string &strSettingPath);

  /**
   * @brief 设置进入仅定位模式
   * @note Use this function if you have deactivated local mapping and you only
   * want to localize the camera.
   * @param flag: 设置仅仅进行跟踪的标志位
   * @return None
   */
  void InformOnlyTracking(const bool &flag);

  /**
   * @brief 整个系统进行复位操作
   * @note
   * @return None
   */
  void Reset();

 public:
  // enum #TODO
  // enum 表示当前追踪线程的状态
  // [-1]SYSTEM_NOT_READY: 系统没有准备好的状态,
  // 一般就是在启动后加载配置文件和词典文件时候的状态 [ 0]NO_IMAGES_YET:
  // 当前无图像 [ 1]NOT_INITIALIZED: 有图像但是没有完成初始化 [ 2]OK:
  // 正常时候的工作状态 [ 3]LOST: 系统已经跟丢了的状态
  enum eTrackingState {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3
  };

  // (eTrackingState) #TODO
  //跟踪状态
  eTrackingState mState;

  // (eTrackingState) #TODO
  //上一帧的跟踪状态.这个变量在绘制当前帧的时候会被使用到
  eTrackingState mLastProcessedState;

  // (int) #TODO
  // Input sensor:MONOCULAR, STEREO, RGBD
  //传感器类型
  int mSensor;

  // (Frame) #TODO
  // Current Frame
  //追踪线程中有一个当前帧
  Frame mCurrentFrame;

  // (cv::Mat) #TODO
  // 还有当前帧的灰度图像
  //? 提问,那么在双目输入和在RGBD输入的时候呢?
  // 在双目输入和在RGBD输入时, 为左侧图像的灰度图
  cv::Mat mImGray;

  // (std::vector<int>) #TODO
  // Initialization Variables (Monocular)
  // 初始化时前两帧相关变量
  //之前的匹配
  std::vector<int> mvIniLastMatches;

  // (std::vector<int>) #TODO
  //初始化阶段中,当前帧中的特征点和参考帧中的特征点的匹配关系
  // 跟踪初始化时前两帧之间的匹配
  std::vector<int> mvIniMatches;

  // (std::vector<cv::Point2f>) #TODO
  //在初始化的过程中,保存参考帧中的特征点
  std::vector<cv::Point2f> mvbPrevMatched;

  // (std::vector<cv::Point3f>) #TODO
  //初始化过程中匹配后进行三角化得到的空间点
  std::vector<cv::Point3f> mvIniP3D;

  // (Frame) #TODO
  //初始化过程中的参考帧
  Frame mInitialFrame;

  // (list<cv::Mat>) #TODO
  // Lists used to recover the full camera trajectory at the end of the
  // execution. Basically we store the reference keyframe for each frame and its
  // relative transformation
  //所有的参考关键帧的位姿;看上面注释的意思,这里存储的也是相对位姿
  list<cv::Mat> mlRelativeFramePoses;

  // (list<KeyFrame *>) #TODO
  //参考关键帧
  list<KeyFrame *> mlpReferences;

  // (list<double>) #TODO
  //所有帧的时间戳  //? 还是关键帧的时间戳?
  list<double> mlFrameTimes;

  // (list<bool>) #TODO
  //是否跟丢的标志
  list<bool> mlbLost;

  // (bool) #TODO
  // True if local mapping is deactivated and we are performing only
  // localization
  //标记当前系统是处于SLAM状态还是纯定位状态
  bool mbOnlyTracking;

 protected:
  /**
   * @brief 主追踪进程
   * @note Main tracking function. It is independent of the input sensor.
   * @return None
   */
  void Track();

  /**
   * @brief 在双目输入和RGBD输入时所做的初始化,主要是产生初始地图
   * @note Map initialization for stereo and RGB-D
   * @return None
   */
  void StereoInitialization();

  /**
   * @brief 单目输入的时候所进行的初始化操作
   * @note Map initialization for monocular
   * @return None
   */
  void MonocularInitialization();

  /**
   * @brief 单目输入的时候生成初始地图
   * @note
   * @return None
   */
  void CreateInitialMapMonocular();

  /**
   * @brief 检查上一帧中的MapPoints是否被替换
   * @note
   * Local Mapping线程可能会将关键帧中某些MapPoints进行替换,
   * 由于tracking中需要用到mLastFrame, 这里检查并更新上一帧中被替换的MapPoints
   * LocalMapping::SearchInNeighbors()
   * @return None
   */
  void CheckReplacedInLastFrame();

  /**
   * @brief 对参考关键帧的MapPoints进行跟踪
   * @note
   * 1. 计算当前帧的词包, 将当前帧的特征点分到特定层的nodes上
   * 2. 对属于同一node的描述子进行匹配
   * 3. 根据匹配对估计当前帧的姿态
   * 4. 根据姿态剔除误匹配
   * @return (bool) 如果匹配数大于10, 返回true
   */
  bool TrackReferenceKeyFrame();

  /**
   * @brief 双目或rgbd摄像头根据深度值为上一帧产生新的MapPoints
   * @note
   * 在双目和rgbd情况下, 选取一些深度小一些的点(可靠一些)
   * 可以通过深度值产生一些新的MapPoints
   * @return None
   */
  void UpdateLastFrame();

  /**
   * @brief 根据匀速度模型对上一帧的MapPoints进行跟踪
   * @note
   * 1. 非单目情况, 需要对上一帧产生一些新的MapPoints(临时)
   * 2. 将上一帧的MapPoints投影到当前帧的图像平面上, 在投影的位置进行区域匹配
   * 3. 根据匹配对估计当前帧的姿态
   * 4. 根据姿态剔除误匹配
   * V-B Initial Pose Estimation From Previous Frame
   * @return (bool) 如果匹配数大于10, 返回true
   */
  bool TrackWithMotionModel();

  /**
   * @brief 重定位模块
   * @note
   * @return (bool) #TODO
   */
  bool Relocalization();

  /**
   * @brief 更新局部地图 LocalMap
   * @note
   * 局部地图包括：共视关键帧、临近关键帧及其子父关键帧,
   * 由这些关键帧观测到的MapPoints
   * @return None
   */
  void UpdateLocalMap();

  /**
   * @brief 更新局部地图点(来自局部关键帧)
   * @note
   * @return None
   */
  void UpdateLocalPoints();

  /**
   * @brief 更新局部关键帧
   * @note
   * 方法是遍历当前帧的MapPoints,
   * 将观测到这些MapPoints的关键帧和相邻的关键帧及其父子关键帧,
   * 作为mvpLocalKeyFrames Step 1：遍历当前帧的MapPoints,
   * 记录所有能观测到当前帧MapPoints的关键帧 Step
   * 2：更新局部关键帧(mvpLocalKeyFrames), 添加局部关键帧有三个策略 Step 2.1
   * 策略1：能观测到当前帧MapPoints的关键帧作为局部关键帧 (将邻居拉拢入伙)
   * Step 2.2 策略2：遍历策略1得到的局部关键帧里共视程度很高的关键帧,
   * 将他们的家人和邻居作为局部关键帧 Step 3：更新当前帧的参考关键帧,
   * 与自己共视程度最高的关键帧作为参考关键帧
   * @return None
   */
  void UpdateLocalKeyFrames();

  /**
   * @brief 对Local Map的MapPoints进行跟踪
   * @note
   * Step 1：更新局部关键帧 mvpLocalKeyFrames 和局部地图点 mvpLocalMapPoints
   * Step 2：在局部地图中查找与当前帧匹配的MapPoints,
   * 其实也就是对局部地图点进行跟踪 Step
   * 3：更新局部所有MapPoints后对位姿再次优化 Step
   * 4：更新当前帧的MapPoints被观测程度, 并统计跟踪局部地图的效果 Step
   * 5：根据跟踪匹配数目及回环情况决定是否跟踪成功
   * @return (bool) true代表跟踪成功; false代表跟踪失败.
   */
  bool TrackLocalMap();

  /**
   * @brief 对 Local MapPoints 进行跟踪
   * @note
   * 在局部地图中查找在当前帧视野范围内的点,
   * 将视野范围内的点和当前帧的特征点进行投影匹配
   * @return None
   */
  void SearchLocalPoints();

  /**
   * @brief 断当前帧是否为关键帧
   * @note
   * @return (bool) true if needed
   */
  bool NeedNewKeyFrame();

  /**
   * @brief 创建新的关键帧
   * @note
   * 对于非单目的情况, 同时创建新的MapPoints
   * @return None
   */
  void CreateNewKeyFrame();

 protected:
  // (bool) #TODO
  // In case of performing only localization, this flag is true when there are
  // no matches to points in the map. Still tracking will continue if there are
  // enough matches with temporal points. In that case we are doing visual
  // odometry. The system will try to do relocalization to recover "zero-drift"
  // localization to the map.
  //当进行纯定位时才会有的一个变量,为false表示该帧匹配了很多的地图点,跟踪是正常的;如果少于10个则为true,表示快要完蛋了
  bool mbVO;

  // (LocalMapping *) #TODO
  // Other Thread Pointers
  //局部建图器句柄
  LocalMapping *mpLocalMapper;

  // (LoopClosing *) #TODO
  //回环检测器句柄
  LoopClosing *mpLoopClosing;

  // (ORBextractor *) #TODO
  // ORB
  // orb特征提取器, 不管单目还是双目, mpORBextractorLeft都要用到
  // 如果是双目, 则要用到mpORBextractorRight
  // NOTICE
  // 如果是单目, 在初始化的时候使用mpIniORBextractor而不是mpORBextractorLeft,
  // mpIniORBextractor属性中提取的特征点个数是mpORBextractorLeft的两倍
  //作者自己编写和改良的ORB特征点提取器
  ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

  // (ORBextractor *) #TODO
  //在初始化的时候使用的特征点提取器,其提取到的特征点个数会更多
  ORBextractor *mpIniORBextractor;

  // (ORBVocabulary *) #TODO
  // BoW 词袋模型相关
  // ORB特征字典
  ORBVocabulary *mpORBVocabulary;

  // (KeyFrameDatabase *) #TODO
  //当前系统运行的时候,关键帧所产生的数据库
  KeyFrameDatabase *mpKeyFrameDB;

  // (Initializer *) #TODO
  // Initalization (only for monocular)
  // 单目初始器
  Initializer *mpInitializer;

  // (KeyFrame *) #TODO
  // Local Map 局部地图相关
  //参考关键帧
  // 当前关键帧就是参考帧
  KeyFrame *mpReferenceKF;

  // (std::vector<KeyFrame *>) #TODO
  //局部关键帧集合
  std::vector<KeyFrame *> mvpLocalKeyFrames;

  // (std::vector<MapPoint *>) #TODO
  //局部地图点的集合
  std::vector<MapPoint *> mvpLocalMapPoints;

  // (System *) #TODO
  // System
  //指向系统实例的指针
  System *mpSystem;

  // (Viewer *) #TODO
  // Drawers  可视化查看器相关
  //查看器对象句柄
  Viewer *mpViewer;

  // (FrameDrawer *) #TODO
  //帧绘制器句柄
  FrameDrawer *mpFrameDrawer;

  // (MapDrawer *) #TODO
  //地图绘制器句柄
  MapDrawer *mpMapDrawer;

  // (Map *) #TODO
  // Map
  //(全局)地图句柄
  Map *mpMap;

  // (cv::Mat) #TODO
  // Calibration matrix   相机的参数矩阵相关
  //相机的内参数矩阵
  cv::Mat mK;

  // (cv::Mat) #TODO
  //相机的去畸变参数
  cv::Mat mDistCoef;

  // (float) #TODO
  //相机的基线长度 * 相机的焦距
  float mbf;

  // (int) #TODO
  // New KeyFrame rules (according to fps)
  // 新建关键帧和重定位中用来判断最小最大时间间隔, 和帧率有关
  int mMinFrames;

  // (int) #TODO
  int mMaxFrames;

  // (float) #TODO
  // Threshold close/far points
  // Points seen as close by the stereo/RGBD sensor are considered reliable
  // and inserted from just one frame. Far points requiere a match in two
  // keyframes.
  //用于区分远点和近点的阈值.
  //近点认为可信度比较高;远点则要求在两个关键帧中得到匹配
  float mThDepth;

  // (float) #TODO
  // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are
  // scaled.
  //深度缩放因子,链接深度值和具体深度值的参数.只对RGBD输入有效
  float mDepthMapFactor;

  // (int) #TODO
  // Current matches in frame
  //当前帧中的进行匹配的内点,将会被不同的函数反复使用
  int mnMatchesInliers;

  // (KeyFrame *) #TODO
  // Last Frame, KeyFrame and Relocalisation Info
  // 上一关键帧
  KeyFrame *mpLastKeyFrame;

  // (Frame) #TODO
  // 上一帧
  Frame mLastFrame;

  // (unsigned int) #TODO
  // 上一个关键帧的ID
  unsigned int mnLastKeyFrameId;

  // (unsigned int) #TODO
  // 上一次重定位的那一帧的ID
  unsigned int mnLastRelocFrameId;

  // (cv::Mat) #TODO
  // Motion Model
  cv::Mat mVelocity;

  // (bool) #TODO
  // Color order (true RGB, false BGR, ignored if grayscale)
  // RGB图像的颜色通道顺序
  bool mbRGB;

  // (list<MapPoint *>) #TODO
  //临时的地图点,用于提高双目和RGBD摄像头的帧间效果,用完之后就扔了
  list<MapPoint *> mlpTemporalPoints;
};

}  // namespace ORB_SLAM2

#endif  // INC_TRACKING_H_
