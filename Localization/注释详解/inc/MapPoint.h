/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-14 14:47:25
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/MapPoint.h
 */

#ifndef INC_MAPPOINT_H_
#define INC_MAPPOINT_H_

// 公用库.
#include <map>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <vector>

// ORB-SLAM2中的其他模块.
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "ORBmatcher.h"

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
class KeyFrame;
class Map;
class Frame;

// 地图点数据类型(一个MapPoint代表一个地图点).
class MapPoint {
 public:
  /**
   * @brief 给定坐标与keyframe构造MapPoint
   * @note 调用情况:
   * 双目:
   * StereoInitialization()
   * CreateNewKeyFrame()
   * LocalMapping::CreateNewMapPoints()
   * 单目:
   * CreateInitialMapMonocular()
   * LocalMapping::CreateNewMapPoints()
   * @param Pos: MapPoint的坐标(wrt世界坐标系)
   * @param pRefKF: KeyFrame
   * @param pMap: Map
   */
  MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap);

  /**
   * @brief 给定坐标与frame构造MapPoint
   * @note 被双目 UpdateLastFrame() 调用
   * @param Pos: MapPoint的坐标(世界坐标系)
   * @param pMap: Map
   * @param pFrame: Frame
   * @param idxF: MapPoint在Frame中的索引, 即对应的特征点的编号
   */
  MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF);

  /**
   * @brief: 设置世界坐标系下地图点的位姿
   * @note note
   * @param Pos: 世界坐标系下地图点的位姿
   * @return None
   */
  void SetWorldPos(const cv::Mat &Pos);

  /**
   * @brief 获取当前地图点在世界坐标系下的位置
   * @note note
   * @return cv::Mat 位置
   */
  cv::Mat GetWorldPos();

  /**
   * @brief 获取当前地图点的平均观测方向
   * @note note
   * @return cv::Mat 一个向量 #TODO
   */
  cv::Mat GetNormal();

  /**
   * @brief 获取生成当前地图点的参考关键帧
   * @note note
   * @return KeyFrame * #TODO
   */
  KeyFrame *GetReferenceKeyFrame();

  /**
   * @brief 获取观测到当前地图点的关键帧
   * @note note
   * @return std::map<KeyFrame*, size_t> 观测到当前地图点的关键帧序列,
   * size_t为这个对象对应为该地图点在该关键帧的特征点的访问id.
   */
  std::map<KeyFrame *, size_t> GetObservations();

  /**
   * @brief: 获取当前地图点的被观测次数
   * @note note
   * @return int #TODO
   */
  int Observations();

  /**
   * @brief 添加观测
   * @note 记录哪些KeyFrame的那个特征点能观测到该MapPoint,
   * 同时增加观测的相机数目nObs, 单目+1, 双目或者grbd+2,
   * 这个函数是建立关键帧共视关系的核心函数,
   * 能共同观测到某些MapPoints的关键帧是共视关键帧.
   * @param pKF: 观测到当前地图点的关键帧
   * @param idx: 在KeyFrame中的索引
   */
  void AddObservation(KeyFrame *pKF, size_t idx);

  /**
   * @brief 取消某个关键帧对当前地图点的观测
   * @note 如果某个关键帧要被删除, 那么会发生这个操作
   * @param pKF: 观测到当前地图点的关键帧 #NOTE: 不一定正确
   */
  void EraseObservation(KeyFrame *pKF);

  /**
   * @brief 获取观测到当前地图点的关键帧,在观测数据中的索引
   * @note
   * @param pKF: 关键帧
   * @return int 索引 #NOTE: 待完善注释
   */
  int GetIndexInKeyFrame(KeyFrame *pKF);

  /**
   * @brief 查看某个关键帧是否看到了当前的地图点
   * @note
   * @param pKF: 关键帧
   * @return bool #TODO
   */
  bool IsInKeyFrame(KeyFrame *pKF);

  /**
   * @brief 告知可以观测到该MapPoint的Frame, 该MapPoint已被删除
   * @note note
   * @return None
   */
  void SetBadFlag();

  /**
   * @brief 判断当前地图点是否是bad
   * @note note
   * @return bool #TODO: true代表, false代表
   */
  bool isBad();

  /**
   * @brief 在形成闭环的时候, 会更新 KeyFrame 与 MapPoint
   * #NOTE: 之间的关系其实也就是相互替换?
   * @note note
   * @param pMP: 地图点
   * @return None
   */
  void Replace(MapPoint *pMP);

  /**
   * @brief 获取取代当前地图点的点? //?
   * @note note
   * @return MapPoint* //?
   */
  MapPoint *GetReplaced();

  /**
   * @brief 增加可视次数
   * @note Visible表示：
   * 1. 该MapPoint在某些帧的视野范围内, 通过Frame::isInFrustum()函数判断
   * 2. 该MapPoint被这些帧观测到, 但并不一定能和这些帧的特征点匹配上
   * 例如：有一个MapPoint(记为M), 在某一帧F的视野范围内,
   * 但并不表明该点M可以和F这一帧的某个特征点能匹配上
   * @param n: 要增加的次数
   * @return None
   */
  void IncreaseVisible(int n = 1);

  /**
   * @brief 能找到该点的帧数+n, n默认为1
   * @param n: 增加的个数
   * #NOTE: 看Tracking::TrackLocalMap()
   */
  void IncreaseFound(int n = 1);

  /**
   * @brief #TODO
   * @note note
   * @return float #TODO
   */
  float GetFoundRatio();

  /**
   * @brief 获取被找到的次数
   * @note note
   * @return int 被找到的次数
   */
  inline int GetFound() { return mnFound; }

  /**
   * @brief 计算具有代表的描述子
   * @note 由于一个MapPoint会被许多相机观测到,
   * 因此在插入关键帧后, 需要判断是否更新当前点的最适合的描述子
   * 先获得当前点的所有描述子, 然后计算描述子之间的两两距离,
   * 最好的描述子与其他描述子应该具有最小的距离中值
   * @return None
   */
  void ComputeDistinctiveDescriptors();

  /**
   * @brief 获取当前地图点的描述子
   * @note note
   * @return cv::Mat #TODO
   */
  cv::Mat GetDescriptor();

  /**
   * @brief 更新平均观测方向以及观测距离范围
   * @note 由于一个MapPoint会被许多相机观测到,
   * 因此在插入关键帧后, 需要更新相应变量
   * @return None
   */
  void UpdateNormalAndDepth();

  /**
   * @brief brief #TODO
   * @note note #TODO
   * @return float #TODO
   */
  float GetMinDistanceInvariance();

  /**
   * @brief brief #TODO
   * @note note #TODO
   * @return float #TODO
   */
  float GetMaxDistanceInvariance();

  /**
   * @brief brief #TODO: 尺度预测?
   * @note note #TODO
   * @return int #TODO
   */
  int PredictScale(const float &currentDist, KeyFrame *pKF);

  /**
   * @brief brief #TODO: 尺度预测?
   * @note note #TODO
   * @return int #TODO
   */
  int PredictScale(const float &currentDist, Frame *pF);

 public:
  // (long unsigned int) 地图点的全局ID
  long unsigned int mnId;  ///< Global ID for MapPoint

  // (static long unsigned int) #TODO
  static long unsigned int nNextId;

  // (const long int) 创建该MapPoint的关键帧ID
  const long int mnFirstKFid;

  // (const long int) 创建该MapPoint的帧ID(即每一关键帧有一个帧ID)
  // 如果是从帧中创建的话,会将普通帧的id存放于这里
  const long int mnFirstFrame;

  // (int) 被观测到的相机数目, 单目+1, 双目或RGBD则+2
  int nObs;

  /**==============================================
   * *                   追踪线程相关变量
   *   TrackLocalMap::SearchByProjection
   *   中决定是否对该点进行投影的变量
   *
   *=============================================**/

  // (float) 当前地图点投影到某帧上后的X坐标
  float mTrackProjX;

  // (float) 当前地图点投影到某帧上后的Y坐标
  float mTrackProjY;

  // (float) 当前地图点投影到某帧上后的X坐标(右目)
  float mTrackProjXR;

  // (int) 所处的尺度, 由其他的类进行操作 #TODO: 不确定
  int mnTrackScaleLevel;

  // (float) 被追踪到时,那帧相机看到当前地图点的视角
  float mTrackViewCos;

  // (bool) 是否在追踪线程视野内 #TODO: 不确定
  // 注意, mbTrackInView==false的点有几种：
  // a. 已经和当前帧经过匹配(TrackReferenceKeyFrame,
  //    TrackWithMotionModel)但在优化过程中认为是外点
  // b. 已经和当前帧经过匹配且为内点,
  //    这类点也不需要再进行投影
  //    #TODO:为什么已经是内点了之后就不需要再进行投影了呢?
  // c. 不在当前相机视野中的点(即未通过isInFrustum判断)
  bool mbTrackInView;

  // (long unsigned int) #TODO
  // TrackLocalMap - UpdateLocalPoints
  // 中防止将MapPoints重复添加至mvpLocalMapPoints的标记
  long unsigned int mnTrackReferenceForFrame;

  // (long unsigned int) #TODO
  // TrackLocalMap - SearchLocalPoints 中决定是否进行isInFrustum判断的变量
  // NOTICE mnLastFrameSeen==mCurrentFrame.mnId的点有几种：
  // a. 已经和当前帧经过匹配(TrackReferenceKeyFrame, TrackWithMotionModel)
  //    但在优化过程中认为是外点
  // b. 已经和当前帧经过匹配且为内点, 这类点也不需要再进行投影
  long unsigned int mnLastFrameSeen;

  // TODO: 下面的....都没看明白

  /**==============================================
   * *                   局部建图相关变量
   *   Variables used by local mapping
   *
   *=============================================**/

  // (long unsigned int) #TODO
  // local mapping中记录地图点对应当前局部BA的关键帧的mnId.mnBALocalForKF 在map
  // point.h里面也有同名的变量.
  long unsigned int mnBALocalForKF;

  // (long unsigned int) #TODO
  // 在局部建图线程中使用,表示被用来进行地图点融合的关键帧(存储的是这个关键帧的id)
  long unsigned int mnFuseCandidateForKF;

  /**==============================================
   * *                   回环检测相关变量
   *   Variables used by loop closing -- 一般都是为了避免重复操作
   *
   *=============================================**/

  // (long unsigned int) #TODO
  // 标记当前地图点是作为哪个"当前关键帧"的回环地图点(即回环关键帧上的地图点),在回环检测线程中被调用
  long unsigned int mnLoopPointForKF;

  // (long unsigned int) #TODO
  // 如果这个地图点对应的关键帧参与到了回环检测的过程中,那么在回环检测过程中已经使用了这个关键帧修正只有的位姿来修正了这个地图点,那么这个标志位置位
  long unsigned int mnCorrectedByKF;

  // (long unsigned int) #TODO
  long unsigned int mnCorrectedReference;

  // (cv::Mat) #TODO
  // 全局BA优化后(如果当前地图点参加了的话),这里记录优化后的位姿
  cv::Mat mPosGBA;

  // (long unsigned int) #TODO
  // 如果当前点的位姿参与到了全局BA优化,那么这个变量记录了那个引起全局BA的"当前关键帧"的id
  long unsigned int mnBAGlobalForKF;

  // (static std::mutex) #TODO
  // 全局BA中对当前点进行操作的时候使用的互斥量
  static std::mutex mGlobalMutex;

 protected:
  // (cv::Mat) #TODO
  // Position in absolute coordinates
  cv::Mat mWorldPos;  ///< MapPoint在世界坐标系下的坐标

  // (std::map<KeyFrame *, size_t>) #TODO
  // Keyframes observing the point and associated index in keyframe
  // 观测到该MapPoint的KF和该MapPoint在KF中的索引
  std::map<KeyFrame *, size_t> mObservations;

  // (cv::Mat) #TODO
  // Mean viewing direction
  // 该MapPoint平均观测方向
  // 用于判断点是否在可视范围内
  cv::Mat mNormalVector;

  // (cv::Mat) #TODO
  // Best descriptor to fast matching
  // 每个3D点也有一个描述子, 但是这个3D点可以观测多个二维特征点,
  // 从中选择一个最有代表性的 通过 ComputeDistinctiveDescriptors()
  // 得到的最有代表性描述子,距离其它描述子的平均距离最小
  cv::Mat mDescriptor;

  // (KeyFrame *) #TODO
  // Reference KeyFrame
  // 通常情况下MapPoint的参考关键帧就是创建该MapPoint的那个关键帧
  KeyFrame *mpRefKF;

  // (int) #TODO
  // Tracking counters
  int mnVisible;

  // (int) #TODO
  // Tracking counters
  int mnFound;

  // (bool) #TODO: 不清楚
  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;

  // (MapPoint *) #TODO: 不清楚
  //? 替换本地图点的点?
  MapPoint *mpReplaced;

  // (float) #TODO: 不清楚
  // Scale invariance distances
  float mfMinDistance;

  // (float) #TODO: 不清楚
  // Scale invariance distances
  float mfMaxDistance;

  // (Map *) #TODO
  //所属的地图
  Map *mpMap;

  // (std::mutex) #TODO
  //对当前地图点位姿进行操作的时候的互斥量
  std::mutex mMutexPos;

  // (std::mutex) #TODO
  // 对当前地图点的特征信息进行操作的时候的互斥量
  std::mutex mMutexFeatures;
};

}  // namespace ORB_SLAM2

#endif  // INC_MAPPOINT_H_
