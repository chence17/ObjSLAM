/*
 * @Author: Antonio Chan
 * @Date: 2021-04-07 22:00:25
 * @LastEditTime: 2021-04-13 12:00:26
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/FrameDrawer.h
 */

#ifndef INC_FRAMEDRAWER_H_
#define INC_FRAMEDRAWER_H_

// 公用库.
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

// ORB-SLAM2中的其他模块.
#include "Map.h"
#include "MapPoint.h"
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
class Viewer;

// 帧绘制器数据类型.
class FrameDrawer {
 public:
  /**
   * @brief 构造函数
   * @note
   * @param pMap: 地图指针
   */
  FrameDrawer(Map *pMap);

  /**
   * @brief 将跟踪线程的数据拷贝到绘图线程( 图像、特征点、地图、跟踪状态)
   * @note Update info from the last processed frame.
   * @param pTracker: 追踪线程
   * @return None
   */
  void Update(Tracking *pTracker);

  /**
   * @brief 绘制最近处理过的帧,这个将会在可视化查看器的窗口中被创建
   * @note Draw last processed frame.
   * @return (cv::Mat) 返回绘制完成的图像,可以直接进行显示
   */
  cv::Mat DrawFrame();

 protected:
  /**
   * @brief 绘制底部的信息栏
   * @note
   * @param im: 原始图像
   * @param nState: 当前系统的工作状态
   * @param imText: 叠加后的图像
   * @return None
   */
  void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

 protected:
  // (cv::Mat) Info of the frame to be drawn
  // 当前绘制的图像
  cv::Mat mIm;

  // (int) 当前帧中特征点的数目
  int N;

  // (vector<cv::KeyPoint>) 当前帧中的特征点
  vector<cv::KeyPoint> mvCurrentKeys;

  // (vector<bool>) 当前帧中的特征点是否在地图中的标记
  // 当前帧的特征点在地图中是否出现;后者是表示地图中没有出现,
  // 但是在当前帧中是第一次被观测得到的点
  vector<bool> mvbMap, mvbVO;

  // (bool) 当前是否是只有追踪线程在工作;
  // 或者说,当前是处于定位模式还是处于SLAM模式
  bool mbOnlyTracking;

  // (int) 当前帧中追踪到的特征点计数
  int mnTracked, mnTrackedVO;

  // (vector<cv::KeyPoint>) 参考帧中的特征点
  vector<cv::KeyPoint> mvIniKeys;

  // (vector<int>) 当前帧特征点和参考帧特征点的匹配关系
  vector<int> mvIniMatches;

  // (int) 当前SLAM系统的工作状态
  int mState;

  // (Map *) 地图指针
  Map *mpMap;

  // (std::mutex) 线程锁
  std::mutex mMutex;
};

}  // namespace ORB_SLAM2

#endif  // INC_FRAMEDRAWER_H_
