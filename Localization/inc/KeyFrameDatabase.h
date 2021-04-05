/*
 * @Author: Antonio Chan
 * @Date: 2021-04-04 21:27:51
 * @LastEditTime: 2021-04-05 12:39:36
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/KeyFrameDatabase.h
 */

#ifndef INC_KEYFRAMEDATABASE_H_
#define INC_KEYFRAMEDATABASE_H_

#include <list>
#include <mutex>
#include <set>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM2 {

class KeyFrame;
class Frame;

class KeyFrameDatabase {
 public:
  KeyFrameDatabase(const ORBVocabulary& voc);

  void add(KeyFrame* pKF);

  void erase(KeyFrame* pKF);

  void clear();

  //检测闭环向量
  // Loop Detection
  std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF, float minScore);

  //检测重定位向量
  // Relocalization
  std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

 protected:
  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

}  // namespace ORB_SLAM2

#endif  // INC_KEYFRAMEDATABASE_H_
