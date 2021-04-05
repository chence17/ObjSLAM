/*
 * @Author: Antonio Chan
 * @Date: 2021-04-04 21:27:51
 * @LastEditTime: 2021-04-04 21:31:59
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/MapDrawer.h
 */

#ifndef INC_MAPDRAWER_H_
#define INC_MAPDRAWER_H_

#include <pangolin/pangolin.h>

#include <mutex>

#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"

namespace ORB_SLAM2 {

class MapDrawer {
 public:
  MapDrawer(Map *pMap, const string &strSettingPath);

  Map *mpMap;

  void DrawMapPoints();
  void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
  void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
  void SetCurrentCameraPose(const cv::Mat &Tcw);
  void SetReferenceKeyFrame(KeyFrame *pKF);
  void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

 private:
  float mKeyFrameSize;
  float mKeyFrameLineWidth;
  float mGraphLineWidth;
  float mPointSize;
  float mCameraSize;
  float mCameraLineWidth;

  cv::Mat mCameraPose;

  std::mutex mMutexCamera;
};

}  // namespace ORB_SLAM2

#endif  // INC_MAPDRAWER_H_