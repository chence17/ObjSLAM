/*
 * @Author: Antonio Chan
 * @Date: 2021-04-07 22:00:25
 * @LastEditTime: 2021-04-14 14:47:46
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/MapDrawer.h
 */

#ifndef INC_MAPDRAWER_H_
#define INC_MAPDRAWER_H_

// 公用库.
#include <pangolin/pangolin.h>

#include <mutex>

// ORB-SLAM2中的其他模块.
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"

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
// 地图可视化模块数据类型.
class MapDrawer {
 public:
  /**
   * @brief 构造函数
   * @note
   * @param pMap: 地图句柄
   * @param strSettingPath: 配置文件的路径
   */
  MapDrawer(Map *pMap, const string &strSettingPath);

  /**
   * @brief 绘制地图点
   * @note
   * @return None
   */
  void DrawMapPoints();

  /**
   * @brief 绘制关键帧
   * @note
   * @param bDrawKF: 是否绘制关键帧
   * @param bDrawGraph: 是否绘制共视图
   * @return None
   */
  void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

  /**
   * @brief 绘制当前相机
   * @note
   * @param Twc: 相机的位姿矩阵
   * @return None
   */
  void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

  /**
   * @brief 设置当前帧的相机位姿
   * @note
   * @param Tcw: 位姿矩阵
   * @return None
   */
  void SetCurrentCameraPose(const cv::Mat &Tcw);

  /**
   * @brief 设置参考关键帧
   * @note
   * @param pKF: 参考关键帧的句柄
   * @return None
   */
  void SetReferenceKeyFrame(KeyFrame *pKF);

  /**
   * @brief 将相机位姿mCameraPose由Mat类型转化为OpenGlMatrix类型
   * @note
   * @param M: #TODO
   * @return None
   */
  void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

 public:
  // (Map *) 地图句柄
  Map *mpMap;

 private:
  // (float) 绘制这些部件的参数
  // 关键帧-大小
  float mKeyFrameSize;

  // (float) 关键帧-线宽
  float mKeyFrameLineWidth;

  // (float) 共视图的线宽
  float mGraphLineWidth;

  // (float) 地图点的大小
  float mPointSize;

  // (float) 绘制的相机的大小
  float mCameraSize;

  // (float) 绘制相机的线宽
  float mCameraLineWidth;

  // (cv::Mat) 相机位置
  cv::Mat mCameraPose;

  // (std::mutex) 线程互斥量
  std::mutex mMutexCamera;
};

}  // namespace ORB_SLAM2

#endif  // INC_MAPDRAWER_H_
