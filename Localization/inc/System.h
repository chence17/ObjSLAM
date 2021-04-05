/*
 * @Author: Antonio Chan
 * @Date: 2021-04-04 21:27:51
 * @LastEditTime: 2021-04-05 18:06:41
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/System.h
 */

#ifndef INC_SYSTEM_H_
#define INC_SYSTEM_H_

#include <opencv2/core/core.hpp>
#include <string>
#include <thread>

#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "Viewer.h"

namespace ORB_SLAM2 {

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System {
 public:
  // eSensor: 枚举类型, 用于表示SLAM系统所使用的传感器类型.
  enum eSensor { MONOCULAR = 0, STEREO = 1, RGBD = 2 };

 public:
  /**
   * @brief: System 构造函数, 用来初始化整个系统.
   * @note:
   * @param strVocFile:
   * * ORB词袋数据.
   * @param strSettingsFile:
   * * 配置文件的路径.
   * @param sensor:
   * * 传感器类型.
   * @param bUseViewer:
   * * 是否显示可视化界面, true 代表显示, false 代表不显示.
   */
  System(const string &strVocFile, const string &strSettingsFile,
         const eSensor sensor, const bool bUseViewer = true);

  // Proccess the given stereo frame. Images must be synchronized and rectified.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  // TODO: 开环VO输出相关, 后端优化控制相关
  /**
   * @brief 双目相机入口(TrackStereo)函数.
   * @note
   * * 每次输入数据以后, 会执行三种操作:
   * * 1) 判断是否需要进入或者退出纯定位模式, 如果需要则执行
   * * 2) 判断是否需要重启tracking, 如果需要则执行
   * * 3) 执行双目相机的跟踪程序.
   * * 注意: 双目图像有同步和校准的概念.
   * @param imLeft:
   * * 左目图像, 彩色图(CV_8UC3)或者灰度图(CV_8U), 彩色图会被自动转换为灰度图.
   * @param imRight:
   * * 右目图像, 彩色图(CV_8UC3)或者灰度图(CV_8U), 彩色图会被自动转换为灰度图.
   * @param timestamp:
   * * 时间戳.
   * @return cv::Mat 输入帧的相机位姿, 如果跟踪失败则为空.
   */
  cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
                      const double &timestamp);

  // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Input depthmap: Float (CV_32F). Returns the camera pose (empty
  // if tracking fails).
  // TODO: 开环VO输出相关, 后端优化控制相关
  /**
   * @description:
   * * RGBD相机入口(TrackRGBD), 每次输入数据以后，会执行三种操作：
   * * 1) 判断是否需要进入或者退出纯定位模式，如果需要则执行
   * * 2) 判断是否需要重启tracking，如果需要则执行
   * * 3) 执行RGBD相机的跟踪程序。
   * @param im:
   * * 视觉图像, 彩色图(CV_8UC3)或者灰度图(CV_8U)
   * @param depthmap:
   * * 深度图像(CV_32F)
   * @param timestamp:
   * * 时间戳
   * @return 输入帧的相机位姿, 如果跟踪失败则为空
   */
  cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap,
                    const double &timestamp);

  // Proccess the given monocular frame
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  // TODO: 开环VO输出相关, 后端优化控制相关
  /**
   * @description:
   * * 单目相机入口(TrackMonocular), 每次输入数据以后，会执行三种操作：
   * * 1) 判断是否需要进入或者退出纯定位模式，如果需要则执行
   * * 2) 判断是否需要重启tracking，如果需要则执行
   * * 3) 执行单目相机的跟踪程序。
   * @param im:
   * * 单目图像, 彩色图(CV_8UC3)或者灰度图(CV_8U)
   * @param timestamp:
   * * 时间戳
   * @return 输入帧的相机位姿, 如果跟踪失败则为空
   */
  cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

  // This stops local mapping thread (map building) and performs only camera
  // tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear map)
  void Reset();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown();

  // Save camera trajectory in the TUM RGB-D dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveTrajectoryTUM(const string &filename);

  // Save keyframe poses in the TUM RGB-D dataset format.
  // This method works for all sensor input.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveKeyFrameTrajectoryTUM(const string &filename);

  // Save camera trajectory in the KITTI dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at:
  // http://www.cvlibs.net/datasets/kitti/eval_odometry.php
  void SaveTrajectoryKITTI(const string &filename);

  // TODO: Save/Load functions
  // SaveMap(const string &filename);
  // LoadMap(const string &filename);

  // Information from most recent processed frame
  // You can call this right after TrackMonocular (or stereo or RGBD)
  int GetTrackingState();
  std::vector<MapPoint *> GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

 private:
  // Input sensor
  eSensor mSensor;

  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary *mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop
  // detection).
  KeyFrameDatabase *mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  Map *mpMap;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints
  // and performs relocalization if tracking fails.
  Tracking *mpTracker;

  // Local Mapper. It manages the local map and performs local bundle
  // adjustment.
  LocalMapping *mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop
  // it performs a pose graph optimization and full bundle adjustment (in a new
  // thread) afterwards.
  LoopClosing *mpLoopCloser;

  // The viewer draws the map and the current camera pose. It uses Pangolin.
  Viewer *mpViewer;

  FrameDrawer *mpFrameDrawer;
  MapDrawer *mpMapDrawer;

  // System threads: Local Mapping, Loop Closing, Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the
  // System object.
  std::thread *mptLocalMapping;
  std::thread *mptLoopClosing;
  std::thread *mptViewer;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  // Tracking state
  int mTrackingState;
  std::vector<MapPoint *> mTrackedMapPoints;
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
  std::mutex mMutexState;
};

}  // namespace ORB_SLAM2

#endif  // INC_SYSTEM_H_
