/*
 * @Author: Antonio Chan
 * @Date: 2021-04-04 21:27:51
 * @LastEditTime: 2021-04-05 16:18:49
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/MapPoint.h
 */

#ifndef INC_MAPPOINT_H_
#define INC_MAPPOINT_H_

#include <mutex>
#include <opencv2/core/core.hpp>

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"

namespace ORB_SLAM2 {

class KeyFrame;
class Map;
class Frame;

/**
 * @description:
 * * MapPoint是地图中的特征点，它自身的参数是三维坐标和描述子，在这个类中它需要完成的主要工作有以下方面：
 * * 1）维护关键帧之间的共视关系
 * * 2）通过计算描述向量之间的距离，在多个关键帧的特征点中找最匹配的特征点
 * * 3）在闭环完成修正后，需要根据修正的主帧位姿修正特征点
 * * 4）对于非关键帧，也产生MapPoint，只不过是给Tracking功能临时使用
 */
class MapPoint {
 public:
  /**
   * @description:
   * * 关键帧相关的地图点构造函数,
   * * 和关键帧相关的地图点构造函数主要是突出地图点和关键帧之间的观测关系,
   * * 参考关键帧是哪一帧, 该地图点被哪些关键帧观测到.
   * * 一个地图点会被多个关键帧观测到,
   * * 多个关键帧之间通过共同观测到地图点而发生的关系叫共视关系, 在ORB-SLAM2中,
   * * 就是通过MapPoint类来维护共视关系的. 在进行局部BA优化时,
   * * 只优化具有共视关系的这些关键帧, 其他关键帧的位姿不参与优化.
   * @param Pos:
   * * 地图点的3D位置
   * @param pRefKF:
   * * 参考关键帧
   * @param pMap:
   * * 地图
   */
  MapPoint(const cv::Mat& Pos, KeyFrame* pRefKF, Map* pMap);

  /**
   * @description:
   * * 普通帧相关的地图点构造函数
   * @param Pos:
   * * 地图点的3D位置
   * @param pMap:
   * * 地图
   * @param pFrame:
   * * 对应的普通帧
   * @param idxF:
   * * 地图点在该帧特征点中的索引号
   */
  MapPoint(const cv::Mat& Pos, Map* pMap, Frame* pFrame, const int& idxF);

  void SetWorldPos(const cv::Mat& Pos);
  cv::Mat GetWorldPos();

  cv::Mat GetNormal();
  KeyFrame* GetReferenceKeyFrame();

  std::map<KeyFrame*, size_t> GetObservations();
  int Observations();

  /**
   * @brief  增加地图点与共视关键帧的观测关系
   * @note
   * * 判断共视关键帧是否已经在观测关系中了，如果是，这里就不会添加；如果不是，往下记录下此关键帧以及此MapPoint的索引，就算是记录下观测信息了
   * @param  pKF: 待添加共视关系的关键帧
   * @param  idx: 该地图点在待添加共视关系的关键帧中对应的索引值
   * @retval None
   */
  void AddObservation(KeyFrame* pKF, size_t idx);
  /**
   * @brief  删除地图点与共视关键帧的观测关系
   * @note
   * * 这个函数首先判断该关键帧是否在观测中，如果在，就从存放观测关系的容器mObservations中移除该关键帧，接着判断该帧是否是参考关键帧，如果是，参考关键帧换成观测的第一帧，因为不能没有参考关键帧呀。删除以后，如果该MapPoint被观测的次数小于2，那么这个MapPoint就没有存在的必要了，需要删除。
   * @param  pKF: 待删除共视关系的关键帧
   * @retval None
   */
  void EraseObservation(KeyFrame* pKF);

  int GetIndexInKeyFrame(KeyFrame* pKF);
  bool IsInKeyFrame(KeyFrame* pKF);

  /**
   * @brief  删除地图点
   * @note
   * * 它的作用就是删除地图点，并清除关键帧和地图中所有和该地图点对应的关联关系
   * @retval None
   */
  void SetBadFlag();
  bool isBad();

  /**
   * @brief  替换地图点
   * @note
   * * 该函数的作用是将当前地图点(this)，替换成pMp，这主要是因为在使用闭环时，完成闭环优化以后，需要调整地图点和关键帧，建立新的关系。
   * * 具体流程是循环遍历所有的观测信息，判断此MapPoint是否在该关键帧中，如果在，那么只要移除原来MapPoint的匹配信息，最后增加这个MapPoint找到的数量以及可见的次数，另外地图中要移除原来的那个MapPoint。最后需要计算这个点独有的描述子。
   * @param  pMP: 用来替换的地图点
   * @retval None
   */
  void Replace(MapPoint* pMP);
  MapPoint* GetReplaced();

  void IncreaseVisible(int n = 1);
  void IncreaseFound(int n = 1);
  float GetFoundRatio();
  inline int GetFound() { return mnFound; }

  /**
   * @brief  计算最匹配的描述子
   * @note
   * * 由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要判断是否更新当前点的最适合的描述子。最好的描述子与其他描述子应该具有最小的平均距离，因此先获得当前点的所有描述子，然后计算描述子之间的两两距离，对所有距离取平均，最后找离这个中值距离最近的描述子。
   * @retval None
   */
  void ComputeDistinctiveDescriptors();

  cv::Mat GetDescriptor();

  /**
   * @brief  更新法向量和深度值
   * @note
   * * 由于图像提取描述子是使用金字塔分层提取，所以计算法向量和深度可以知道该MapPoint在对应的关键帧的金字塔哪一层可以提取到。
   * * 明确了目的，下一步就是方法问题，所谓的法向量，就是也就是说相机光心指向地图点的方向，计算这个方向方法很简单，只需要用地图点的三维坐标减去相机光心的三维坐标就可以。
   * @retval None
   */
  void UpdateNormalAndDepth();

  float GetMinDistanceInvariance();
  float GetMaxDistanceInvariance();

  // 下图中横线的大小表示不同图层图像上的一个像素表示的真实物理空间中的大小
  //              ____
  // Nearer      /____\     level:n-1 --> dmin
  //            /______\                       d/dmin = 1.2^(n-1-m)
  //           /________\   level:m   --> d
  //          /__________\                     dmax/d = 1.2^m
  // Farther /____________\ level:0   --> dmax
  //
  //           log(dmax/d)
  // m = ceil(------------)
  //            log(1.2)
  // 这个函数的作用:
  // 在进行投影匹配的时候会给定特征点的搜索范围,考虑到处于不同尺度(也就是距离相机远近,位于图像金字塔中不同图层)的特征点受到相机旋转的影响不同,
  // 因此会希望距离相机近的点的搜索范围更大一点,距离相机更远的点的搜索范围更小一点,所以要在这里,根据点到关键帧/帧的距离来估计它在当前的关键帧/帧中,
  // 会大概处于哪个尺度

  /**
   * @brief  预测尺度
   * @note   该函数的作用是预测特征点在金字塔哪一层可以找到。
   * @param  currentDist: 相机光心距离地图点距离
   * @param  pKF: 关键帧
   * @return int 预测的金字塔尺度
   */
  int PredictScale(const float& currentDist, KeyFrame* pKF);
  int PredictScale(const float& currentDist, Frame* pF);

 public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;
  // nObs用来记录被观测的次数
  int nObs;

  // Variables used by the tracking
  float mTrackProjX;
  float mTrackProjY;
  float mTrackProjXR;
  bool mbTrackInView;
  int mnTrackScaleLevel;
  float mTrackViewCos;
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;
  cv::Mat mPosGBA;
  long unsigned int mnBAGlobalForKF;

  static std::mutex mGlobalMutex;

 protected:
  // Position in absolute coordinates
  cv::Mat mWorldPos;

  // mObservations用来存放观测关系的容器，把能够观测到该MapPoint的关键帧，以及MapPoint在该关键帧中对应的索引值关联并存储起来
  // Keyframes observing the point and associated index in keyframe
  std::map<KeyFrame*, size_t> mObservations;

  // Mean viewing direction
  cv::Mat mNormalVector;

  // Best descriptor to fast matching
  cv::Mat mDescriptor;

  // Reference KeyFrame
  KeyFrame* mpRefKF;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;
  MapPoint* mpReplaced;

  // Scale invariance distances
  float mfMinDistance;
  float mfMaxDistance;

  Map* mpMap;

  std::mutex mMutexPos;
  std::mutex mMutexFeatures;
};

}  // namespace ORB_SLAM2

#endif  // INC_MAPPOINT_H_
