/*
 * @Author: Antonio Chan
 * @Date: 2021-04-07 22:00:25
 * @LastEditTime: 2021-04-13 11:24:32
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/Viewer.h
 */

#ifndef INC_VIEWER_H_
#define INC_VIEWER_H_

// 公用库.
#include <mutex>

// ORB-SLAM2中的其他模块.
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "System.h"
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
class FrameDrawer;
class MapDrawer;
class System;

// 可视化模块数据类型.
class Viewer {
 public:
  /**
   * @brief 构造函数
   * @note
   * @param pSystem: 系统实例
   * @param pFrameDrawer: 帧绘制器
   * @param pMapDrawer: 地图绘制器
   * @param pTracking: 追踪线程
   * @param strSettingPath: 设置文件的路径
   */
  Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer,
         Tracking* pTracking, const string& strSettingPath);

  /**
   * @brief 进程的主函数
   * @note 注意到这里提到了它是根据相机图像的更新帧率来绘制图像的
   * Main thread function. Draw points, keyframes, the current camera
   * pose and the last processed frame. Drawing is refreshed according to the
   * camera fps. We use Pangolin.
   * @return None
   */
  void Run();

  /**
   * @brief 请求停止当前进程
   * @note
   * @return None
   */
  void RequestFinish();

  /**
   * @brief 请求当前可视化进程暂停更新图像数据
   * @note
   * @return None
   */
  void RequestStop();

  /**
   * @brief 当前是否有停止当前进程的请求
   * @note
   * @return (bool) #TODO
   */
  bool isFinished();

  /**
   * @brief 判断当前进程是否已经停止
   * @note
   * @return (bool) #TODO
   */
  bool isStopped();

  /**
   * @brief 释放变量, 避免互斥关系
   * @note
   * @return None
   */
  void Release();

 private:
  /**
   * @brief 停止当前查看器的更新
   * @note
   * @return (bool) true代表成功停止, false代表失败,
   * 一般是因为查看器进程已经销毁或者是正在销毁
   */
  bool Stop();

  /**
   * @brief 检查当前查看器进程是否已经终止
   * @note
   * @return (bool) #TODO
   */
  bool CheckFinish();

  /**
   * @brief 设置当前线程终止
   * @note
   * @return None
   */
  void SetFinish();

 private:
  // (System*) 系统对象指针
  System* mpSystem;

  // (FrameDrawer*) 帧绘制器
  FrameDrawer* mpFrameDrawer;

  // (MapDrawer*) 地图绘制器
  MapDrawer* mpMapDrawer;

  // (Tracking*) 追踪线程句柄
  Tracking* mpTracker;

  // (double) 1/fps in ms
  // 每一帧图像持续的时间
  double mT;

  // (float) 图像的尺寸
  float mImageWidth, mImageHeight;

  // (float) 显示窗口的的查看视角,最后一个是相机的焦距
  float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

  // (bool) 请求结束当前线程的标志
  bool mbFinishRequested;

  // (bool) 当前线程是否已经终止
  bool mbFinished;

  // (std::mutex) 线程锁对象,用于锁住和finsh,终止当前查看器进程相关的变量
  //?
  //但是我现在还是不明白,它是怎么知道我的这个线程锁对象和我的这个线程产生绑定关系的
  std::mutex mMutexFinish;

  // (bool) 当前进程是否停止
  bool mbStopped;

  // (bool) 是否头停止请求
  bool mbStopRequested;

  // (std::mutex) 用于锁住stop,停止更新变量相关的互斥量
  std::mutex mMutexStop;
};

}  // namespace ORB_SLAM2

#endif  // INC_VIEWER_H_
