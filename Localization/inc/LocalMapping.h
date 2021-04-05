/*
 * @Author: Antonio Chan
 * @Date: 2021-04-04 21:27:51
 * @LastEditTime: 2021-04-04 21:32:13
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/LocalMapping.h
 */

#ifndef INC_LOCALMAPPING_H_
#define INC_LOCALMAPPING_H_

#include <mutex>

#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LoopClosing.h"
#include "Map.h"
#include "Tracking.h"

namespace ORB_SLAM2 {

class Tracking;
class LoopClosing;
class Map;

class LocalMapping {
 public:
  LocalMapping(Map* pMap, const float bMonocular);

  void SetLoopCloser(LoopClosing* pLoopCloser);

  void SetTracker(Tracking* pTracker);

  // Main function
  void Run();

  void InsertKeyFrame(KeyFrame* pKF);

  // Thread Synch
  void RequestStop();
  void RequestReset();
  bool Stop();
  void Release();
  bool isStopped();
  bool stopRequested();
  bool AcceptKeyFrames();
  void SetAcceptKeyFrames(bool flag);
  bool SetNotStop(bool flag);

  void InterruptBA();

  void RequestFinish();
  bool isFinished();

  int KeyframesInQueue() {
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlNewKeyFrames.size();
  }

 protected:
  bool CheckNewKeyFrames();
  void ProcessNewKeyFrame();
  void CreateNewMapPoints();

  void MapPointCulling();
  void SearchInNeighbors();

  void KeyFrameCulling();

  cv::Mat ComputeF12(KeyFrame*& pKF1, KeyFrame*& pKF2);

  cv::Mat SkewSymmetricMatrix(const cv::Mat& v);

  bool mbMonocular;

  void ResetIfRequested();
  bool mbResetRequested;
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  Map* mpMap;

  LoopClosing* mpLoopCloser;
  Tracking* mpTracker;

  std::list<KeyFrame*> mlNewKeyFrames;

  KeyFrame* mpCurrentKeyFrame;

  std::list<MapPoint*> mlpRecentAddedMapPoints;

  std::mutex mMutexNewKFs;

  bool mbAbortBA;

  bool mbStopped;
  bool mbStopRequested;
  bool mbNotStop;
  std::mutex mMutexStop;

  bool mbAcceptKeyFrames;
  std::mutex mMutexAccept;
};

}  // namespace ORB_SLAM2

#endif  // INC_LOCALMAPPING_H_
