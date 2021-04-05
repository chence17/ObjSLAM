/*
 * @Author: Antonio Chan
 * @Date: 2021-04-04 21:27:51
 * @LastEditTime: 2021-04-04 21:32:33
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/FrameDrawer.h
 */

#ifndef INC_FRAMEDRAWER_H_
#define INC_FRAMEDRAWER_H_

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Map.h"
#include "MapPoint.h"
#include "Tracking.h"

namespace ORB_SLAM2 {

class Tracking;
class Viewer;

class FrameDrawer {
 public:
  FrameDrawer(Map *pMap);

  // Update info from the last processed frame.
  void Update(Tracking *pTracker);

  // Draw last processed frame.
  cv::Mat DrawFrame();

 protected:
  void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

  // Info of the frame to be drawn
  cv::Mat mIm;
  int N;
  vector<cv::KeyPoint> mvCurrentKeys;
  vector<bool> mvbMap, mvbVO;
  bool mbOnlyTracking;
  int mnTracked, mnTrackedVO;
  vector<cv::KeyPoint> mvIniKeys;
  vector<int> mvIniMatches;
  int mState;

  Map *mpMap;

  std::mutex mMutex;
};

}  // namespace ORB_SLAM2

#endif  // INC_FRAMEDRAWER_H_
