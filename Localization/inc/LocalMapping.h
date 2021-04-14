/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-14 14:49:12
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/LocalMapping.h
 */

#ifndef INC_LOCALMAPPING_H_
#define INC_LOCALMAPPING_H_

// 公用库.
#include <map>
#include <mutex>
#include <vector>

// ORB-SLAM2中的其他模块.
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LoopClosing.h"
#include "Map.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Tracking.h"

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
class Tracking;
class LoopClosing;
class Map;

// 局部建图线程数据类型.
class LocalMapping {
 public:
  /**
   * @brief 局部建图线程数据类型的构造函数.
   * @note
   * @param pMap: 局部地图的句柄？ #TODO
   * @param bMonocular: 当前系统是否是单目输入
   */
  LocalMapping(Map *pMap, const float bMonocular);

  /**
   * @brief 设置回环检测线程句柄
   * @note
   * @param pLoopCloser: 回环检测线程句柄
   * @return None
   */
  void SetLoopCloser(LoopClosing *pLoopCloser);

  /**
   * @brief 设置追踪线程句柄
   * @note
   * @param pTracker: 追踪线程句柄
   * @return None
   */
  void SetTracker(Tracking *pTracker);

  /**
   * @brief 线程主函数
   * @note Main function
   * @return None
   */
  void Run();

  /**
   * @brief 插入关键帧,由外部线程调用
   * @note 将关键帧插入到地图中, 以便将来进行局部地图优化
   * NOTICE 这里仅仅是将关键帧插入到列表中进行等待
   * @param pKF: KeyFrame
   * @return None
   */
  void InsertKeyFrame(KeyFrame *pKF);

  /**
   * @brief 外部线程调用,请求停止当前线程的工作
   * @note Thread Synch
   * @return None
   */
  void RequestStop();

  /**
   * @brief 请求当前线程复位,由外部线程调用,堵塞的
   * @note
   * @return None
   */
  void RequestReset();

  /**
   * @brief 停止线程
   * @note
   * 检查是否要把当前的局部建图线程停止,
   * 如果当前线程没有那么检查请求标志,
   * 如果请求标志被置位那么就设置为停止工作.
   * 由run函数调用
   * @return (bool) #TODO
   */
  bool Stop();

  /**
   * @brief 释放当前还在缓冲区中的关键帧指针
   * @note
   * @return None
   */
  void Release();

  /**
   * @brief 检查mbStopped是否被置位了
   * @note
   * @return (bool) #TODO
   */
  bool isStopped();

  /**
   * @brief 是否有终止当前线程的请求
   * @note
   * @return (bool) #TODO
   */
  bool stopRequested();

  /**
   * @brief 查看当前是否允许接受关键帧
   * @note
   * @return (bool) #TODO
   */
  bool AcceptKeyFrames();

  /**
   * @brief 设置"允许接受关键帧"的状态标志
   * @note
   * @param flag: 是或者否
   * @return None
   */
  void SetAcceptKeyFrames(bool flag);

  /**
   * @brief 设置 mbnotStop标志的状态
   * @note
   * @param flag: 是或者否
   * @return (bool) #TODO
   */
  bool SetNotStop(bool flag);

  /**
   * @brief 外部线程调用,终止BA
   * @note
   * @return None
   */
  void InterruptBA();

  /**
   * @brief 请求终止当前线程
   * @note
   * @return None
   */
  void RequestFinish();

  /**
   * @brief 当前线程的run函数是否已经终止
   * @note
   * @return (bool) #TODO
   */
  bool isFinished();

  /**
   * @brief 查看队列中等待插入的关键帧数目
   * @note
   * @return (int) 关键帧数目
   */
  int KeyframesInQueue() {
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlNewKeyFrames.size();
  }

 protected:
  /**
   * @brief 查看列表中是否有等待被插入的关键帧
   * @note
   * @return (bool) 如果存在, 返回true
   */
  bool CheckNewKeyFrames();

  /**
   * @brief 处理列表中的关键帧
   * @note
   * - 计算Bow, 加速三角化新的MapPoints
   * - 关联当前关键帧至MapPoints, 并更新MapPoints的平均观测方向和观测距离范围
   * - 插入关键帧, 更新Covisibility图和Essential图
   * VI-A keyframe insertion
   * @return None
   */
  void ProcessNewKeyFrame();

  /**
   * @brief 相机运动过程中和共视程度比较高的关键帧通过三角化恢复出一些MapPoints
   * @note
   * @return None
   */
  void CreateNewMapPoints();

  /**
   * @brief 剔除ProcessNewKeyFrame和CreateNewMapPoints函数中
   * 引入的质量不好的MapPoints
   * @note VI-B recent map points culling
   * @return None
   */
  void MapPointCulling();

  /**
   * @brief 检查并融合当前关键帧与相邻帧(两级相邻)重复的MapPoints
   * @note
   * @return None
   */
  void SearchInNeighbors();

  /**
   * @brief 关键帧剔除
   * @note 在Covisibility Graph中的关键帧,
   * 其90%以上的MapPoints能被其他关键帧(至少3个)观测到,
   * 则认为该关键帧为冗余关键帧.
   * VI-E Local Keyframe Culling
   * @return None
   */
  void KeyFrameCulling();

  /**
   * 根据两关键帧的姿态计算两个关键帧之间的基本矩阵
   * @param pKF1: 关键帧1
   * @param pKF2: 关键帧2
   * @return (cv::Mat) 基本矩阵
   */
  cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

  /**
   * @brief 计算三维向量v的反对称矩阵
   * @note
   * @param v: 三维向量
   * @return (cv::Mat) 反对称矩阵
   */
  cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

  /**
   * @brief 检查当前是否有复位线程的请求
   * @note
   * @return None
   */
  void ResetIfRequested();

  /**
   * @brief 检查是否已经有外部线程请求终止当前线程
   * @note
   * @return (bool) #TODO
   */
  bool CheckFinish();

  /**
   * @brief 设置当前线程已经真正地结束了,由本线程run函数调用
   * @note
   * @return None
   */
  void SetFinish();

 protected:
  // (bool) #TODO
  // 当前系统输入数单目还是双目RGB-D的标志
  bool mbMonocular;

  // (bool) #TODO
  // 当前系统是否收到了请求复位的信号
  bool mbResetRequested;

  // (std::mutex) #TODO
  // 和复位信号有关的互斥量
  std::mutex mMutexReset;

  // (bool) #TODO
  // 当前线程是否收到了请求终止的信号
  bool mbFinishRequested;

  // (bool) #TODO
  // 当前线程的主函数是否已经终止
  bool mbFinished;

  // (std::mutex) #TODO
  // 和"线程真正结束"有关的互斥锁
  std::mutex mMutexFinish;

  // (Map *) #TODO
  // 指向局部地图的句柄
  Map *mpMap;

  // (LoopClosing *) #TODO
  // 回环检测线程句柄
  LoopClosing *mpLoopCloser;

  // (Tracking *) #TODO
  // 追踪线程句柄
  Tracking *mpTracker;

  // (std::list<KeyFrame *>) #TODO
  // Tracking线程向LocalMapping中插入关键帧是先插入到该队列中
  // 等待处理的关键帧列表
  std::list<KeyFrame *> mlNewKeyFrames;

  // (KeyFrame *) #TODO
  // 当前正在处理的关键帧
  KeyFrame *mpCurrentKeyFrame;

  // (std::list<MapPoint *>) #TODO
  // 存储当前关键帧生成的地图点,也是等待检查的地图点列表
  std::list<MapPoint *> mlpRecentAddedMapPoints;

  // (std::mutex) #TODO
  // 操作关键帧列表时使用的互斥量
  std::mutex mMutexNewKFs;

  // (bool) #TODO
  // 终止BA的标志
  bool mbAbortBA;

  // (bool) #TODO
  // 当前线程是否已经真正地终止了
  bool mbStopped;

  // (bool) #TODO
  // 终止当前线程的请求
  bool mbStopRequested;

  // (bool) #TODO
  // 标志这当前线程还不能够停止工作,优先级比那个"mbStopRequested"要高.只有这个和mbStopRequested都满足要求的时候,线程才会进行一系列的终止操作
  bool mbNotStop;

  // (std::mutex) #TODO
  // 和终止线程相关的互斥锁
  std::mutex mMutexStop;

  // (bool) #TODO
  // 当前局部建图线程是否允许关键帧输入
  bool mbAcceptKeyFrames;

  // (std::mutex) #TODO
  // 和操作上面这个变量有关的互斥量
  std::mutex mMutexAccept;
};

}  // namespace ORB_SLAM2

#endif  // INC_LOCALMAPPING_H_
