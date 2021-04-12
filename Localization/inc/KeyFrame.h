/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-12 22:19:19
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/KeyFrame.h
 */

#ifndef INC_KEYFRAME_H_
#define INC_KEYFRAME_H_

// 公用库.
#include <map>
#include <mutex>
#include <vector>

#include "third_party/DBoW2/DBoW2/BowVector.h"
#include "third_party/DBoW2/DBoW2/FeatureVector.h"

// ORB-SLAM2中的其他模块.
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

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
class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

// 关键帧数据类型.
// 关键帧和普通帧不一样, 但是可以由Frame来构造,
// 许多数据会被三个线程同时访问, 所以用锁的地方很普遍.
class KeyFrame {
 public:
  /**
   * @brief 构造函数
   * @note
   * @param F: 父类普通帧的对象
   * @param pMap: 所属的地图指针
   * @param pKFDB: 使用的词袋模型的指针
   */
  KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

  /**==============================================
   * *                   Pose functions
   *   这里的set,get需要用到锁
   *
   *=============================================**/

  /**
   * @brief 设置当前关键帧的位姿
   * @note
   * @param Tcw: 位姿
   * @return None
   */
  void SetPose(const cv::Mat &Tcw);

  /**
   * @brief 获取位姿
   * @note note
   * @return (cv::Mat) 位姿
   */
  cv::Mat GetPose();

  /**
   * @brief 获取位姿的逆
   * @note note
   * @return (cv::Mat) 位姿的逆
   */
  cv::Mat GetPoseInverse();

  /**
   * @brief 获取(左目)相机的中心
   * @note note
   * @return (cv::Mat) (左目)相机的中心
   */
  cv::Mat GetCameraCenter();

  /**
   * @brief 获取双目相机的中心
   * @note 这个只有在可视化的时候才会用到
   * @return (cv::Mat) 双目相机的中心
   */
  cv::Mat GetStereoCenter();

  /**
   * @brief 获取姿态
   * @note note
   * @return (cv::Mat) 姿态
   */
  cv::Mat GetRotation();

  /**
   * @brief 获取位置
   * @note note
   * @return (cv::Mat) 位置
   */
  cv::Mat GetTranslation();

  /**
   * @brief Bag of Words Representation
   * @note
   * 计算mBowVec, 并且将描述子分散在第4层上,
   * 即mFeatVec记录了属于第i个node的ni个描述子
   * ProcessNewKeyFrame()
   * @return None
   */
  void ComputeBoW();

  /**==============================================
   * *                   共视图函数
   *   Covisibility graph functions
   *
   *=============================================**/

  /**
   * @brief 为关键帧之间添加连接
   * @note 更新了mConnectedKeyFrameWeights
   * @param pKF: 关键帧
   * @param weight: 权重, 该关键帧与pKF共同观测到的3d点数量
   * @return None
   */
  void AddConnection(KeyFrame *pKF, const int &weight);

  /**
   * @brief 删除当前关键帧和指定关键帧之间的共视关系
   * @note
   * @param pKF: 要删除的共视关系
   * @return None
   */
  void EraseConnection(KeyFrame *pKF);

  /**
   * @brief 更新图的连接
   * @note
   * @return None
   */
  void UpdateConnections();

  /**
   * @brief 按照权重对连接的关键帧进行排序
   * @note 更新后的变量存储在mvpOrderedConnectedKeyFrames和mvOrderedWeights中
   * @return None
   */
  void UpdateBestCovisibles();

  /**
   * @brief 得到与该关键帧连接的关键帧(没有排序的)
   * @note
   * @return (std::set<KeyFrame *>) 连接的关键帧
   */
  std::set<KeyFrame *> GetConnectedKeyFrames();

  /**
   * @brief 得到与该关键帧连接的关键帧(已按权值排序)
   * @note
   * @return (std::vector<KeyFrame *>) 连接的关键帧
   */
  std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();

  /**
   * @brief 得到与该关键帧连接的前N个关键帧(已按权值排序)
   * @note NOTICE
   * 如果连接的关键帧少于N,
   * 则返回所有连接的关键帧,所以说返回的关键帧的数目其实不一定是N个
   * @param N: 前N个
   * @return (std::vector<KeyFrame *>) 连接的关键帧
   */
  std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);

  /**
   * @brief 得到与该关键帧连接的权重大于等于w的关键帧
   * @note
   * @param w: 权重
   * @return (std::vector<KeyFrame *>) 连接的关键帧
   */
  std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);

  /**
   * @brief 得到该关键帧与pKF的权重
   * @note
   * @param  pKF: 关键帧
   * @return (int) 权重
   */
  int GetWeight(KeyFrame *pKF);

  /**==============================================
   * *                   旋转树函数
   *   Spanning tree functions
   *
   *=============================================**/

  /**
   * @brief 添加子关键帧(即和子关键帧具有最大共视关系的关键帧就是当前关键帧)
   * @note
   * @param pKF: 子关键帧句柄
   * @return None
   */
  void AddChild(KeyFrame *pKF);

  /**
   * @brief 删除某个子关键帧
   * @note
   * @param pKF: 子关键帧句柄
   * @return None
   */
  void EraseChild(KeyFrame *pKF);

  /**
   * @brief 改变当前关键帧的父关键帧
   * @note
   * @param pKF: 父关键帧句柄
   * @return None
   */
  void ChangeParent(KeyFrame *pKF);

  /**
   * @brief 获取获取当前关键帧的子关键帧
   * @note
   * @return (std::set<KeyFrame*>) 子关键帧集合
   */
  std::set<KeyFrame *> GetChilds();

  /**
   * @brief 获取当前关键帧的父关键帧
   * @note
   * @return (KeyFrame *) 父关键帧句柄
   */
  KeyFrame *GetParent();

  /**
   * @brief 判断某个关键帧是否是当前关键帧的子关键帧
   * @note
   * @param pKF: 关键帧句柄
   * @return (bool) #TODO
   */
  bool hasChild(KeyFrame *pKF);

  /**==============================================
   * *                   回环边函数
   *   Loop Edges
   *
   *=============================================**/

  /**
   * @brief 给当前关键帧添加回环边, 回环边连接了形成闭环关系的关键帧
   * @note
   * @param pKF: 和当前关键帧形成闭环关系的关键帧
   * @return None
   */
  void AddLoopEdge(KeyFrame *pKF);

  /**
   * @brief 获取和当前关键帧形成闭环关系的关键帧
   * @note
   * @return (std::set<KeyFrame*>) 结果
   */
  std::set<KeyFrame *> GetLoopEdges();

  /**==============================================
   * *                   地图点观测函数
   *   MapPoint observation functions
   *
   *=============================================**/

  /**
   * @brief Add MapPoint to KeyFrame
   * @note
   * @param pMP: MapPoint
   * @param idx: MapPoint在KeyFrame中的索引
   * @return None
   */
  void AddMapPoint(MapPoint *pMP, const size_t &idx);

  /**
   * @brief
   * @note
   * 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,
   * 这里是"通知"当前关键帧这个地图点已经被删除了
   * @param idx: 被删除的地图点索引
   * @return None
   */
  void EraseMapPointMatch(const size_t &idx);

  /**
   * @brief
   * @note
   * 由于其他的原因,导致当前关键帧观测到的某个地图点被删除(bad==true)了,
   * 这里是"通知"当前关键帧这个地图点已经被删除了
   * @param pMP: 被删除的地图点指针
   * @return None
   */
  void EraseMapPointMatch(MapPoint *pMP);

  /**
   * @brief 地图点的替换
   * @note
   * @param idx: 要替换掉的地图点的索引
   * @param pMP: 新地图点的指针
   * @return None
   */
  void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);

  /**
   * @brief 获取当前帧中的所有地图点
   * @note
   * @return std::set<MapPoint*> 所有的地图点
   */
  std::set<MapPoint *> GetMapPoints();

  /**
   * @brief Get MapPoint Matches 获取该关键帧的MapPoints
   * @note
   * @return (std::vector<MapPoint *>) #TODO
   */
  std::vector<MapPoint *> GetMapPointMatches();

  /**
   * @brief 关键帧中, 大于等于minObs的MapPoints的数量
   * @note
   * minObs就是一个阈值, 大于minObs就表示该MapPoint是一个高质量的MapPoint
   * 一个高质量的MapPoint会被多个KeyFrame观测到.
   * @param minObs: 最小观测
   * @return (int) #TODO
   */
  int TrackedMapPoints(const int &minObs);

  /**
   * @brief 获取获取当前关键帧的具体的某个地图点
   * @note
   * @param idx: id
   * @return (MapPoint *) 地图点句柄
   */
  MapPoint *GetMapPoint(const size_t &idx);

  /**==============================================
   * *                   关键点函数
   *   Key Point functions
   *
   *=============================================**/

  /**
   * @brief 获取某个特征点的邻域中的特征点id
   * @note
   * @param x: 特征点坐标
   * @param y: 特征点坐标
   * @param r: 邻域大小(半径)
   * @return (std::vector<size_t>) 在这个邻域内找到的特征点索引的集合
   */
  std::vector<size_t> GetFeaturesInArea(const float &x, const float &y,
                                        const float &r) const;
  /**
   * @brief Backprojects a keypoint (if stereo/depth info available) into 3D
   * @note world coordinates.
   * @param i: 第i个keypoint
   * @return (cv::Mat) 3D点(相对于世界坐标系)
   */
  cv::Mat UnprojectStereo(int i);

  /**
   * @brief 判断某个点是否在当前关键帧的图像中
   * @note
   * @param x: 点的坐标
   * @param y: 点的坐标
   * @return (bool) #TODO
   */
  bool IsInImage(const float &x, const float &y) const;

  /**
   * @brief 设置当前关键帧不要在优化的过程中被删除
   * @note Enable/Disable bad flag changes
   * @return None
   */
  void SetNotErase();

  /**
   * @brief 准备删除当前的这个关键帧,表示不进行回环检测过程;由回环检测线程调用
   * @note
   * @return None
   */
  void SetErase();

  /**
   * @brief 真正地执行删除关键帧的操作
   * @note Set/check bad flag
   * @return None
   */
  void SetBadFlag();

  /**
   * @brief 返回当前关键帧是否已经完蛋了
   * @note
   * @return (bool) #TODO
   */
  bool isBad();

  /**
   * @brief 评估当前关键帧场景深度, q=2表示中值
   * @note Compute Scene Depth (q=2 median). Used in monocular.
   * @param q: q=2
   * @return (float) Median Depth
   */
  float ComputeSceneMedianDepth(const int q);

  /**
   * @brief 比较两个int型权重的大小的比较函数
   * @note note
   * @param a: #TODO
   * @param b: #TODO
   * @return (bool) #TODO
   */
  static bool weightComp(int a, int b) { return a > b; }

  /**
   * @brief #TODO
   * @note note
   * @param pKF1: #TODO
   * @param pKF2: #TODO
   * @return (bool) #TODO
   */
  static bool lId(KeyFrame *pKF1, KeyFrame *pKF2) {
    return pKF1->mnId < pKF2->mnId;
  }

 public:
  // (static long unsigned int) #TODO
  // The following variables are accesed from only 1 thread or never change (no
  // mutex needed).
  // nNextID名字改为nLastID更合适, 表示上一个KeyFrame的ID号
  static long unsigned int nNextId;

  // (long unsigned int) #TODO
  // 在nNextID的基础上加1就得到了mnID, 为当前KeyFrame的ID号
  long unsigned int mnId;

  // (const long unsigned int) #TODO
  // 每个KeyFrame基本属性是它是一个Frame, KeyFrame初始化的时候需要Frame,
  // mnFrameId记录了该KeyFrame是由哪个Frame初始化的
  const long unsigned int mnFrameId;

  // (const double unsigned int) #TODO
  // 时间戳
  const double mTimeStamp;

  // (const int) #TODO
  // Grid (to speed up feature matching)
  // 和Frame类中的定义相同
  const int mnGridCols;

  // (const int) #TODO
  const int mnGridRows;

  // (const float) #TODO
  const float mfGridElementWidthInv;

  // (const float) #TODO
  const float mfGridElementHeightInv;

  // (long unsigned int) #TODO
  // Variables used by the tracking
  // 记录它
  long unsigned int mnTrackReferenceForFrame;

  // (long unsigned int) #TODO
  // 标记在局部建图线程中,和哪个关键帧进行融合的操作
  long unsigned int mnFuseTargetForKF;

  // (long unsigned int) #TODO
  // Variables used by the local mapping
  // local
  // mapping中记录当前处理的关键帧的mnId,
  // 表示当前局部BA的关键帧id.mnBALocalForKF 在map point.h里面也有同名的变量.
  long unsigned int mnBALocalForKF;

  // (long unsigned int) #TODO
  // local mapping中记录当前处理的关键帧的mnId,
  // 只是提供约束信息但是却不会去优化这个关键帧
  long unsigned int mnBAFixedForKF;

  // (long unsigned int) #TODO
  // Variables used by the keyframe database
  // 下面的这些变量都是临时的,由外部调用暂时存放一些数据
  // 标记了当前关键帧是id为mnLoopQuery的回环检测的候选关键帧
  long unsigned int mnLoopQuery;

  // (int) #TODO
  // 当前关键帧和这个形成回环的候选关键帧中,具有相同word的个数
  int mnLoopWords;

  // (float) #TODO
  // 和那个形成回环的关键帧的词袋匹配程度的评分
  float mLoopScore;

  // (long unsigned int) #TODO
  // 用来存储在辅助进行重定位的时候, 要进行重定位的那个帧的id
  long unsigned int mnRelocQuery;

  // (int) #TODO
  // 和那个要进行重定位的帧,所具有相同的单词的个数
  int mnRelocWords;

  // (float) #TODO
  // 还有和那个帧的词袋的相似程度的评分
  float mRelocScore;

  // (cv::Mat) #TODO
  // Variables used by loop closing
  // 经过全局BA优化后的相机的位姿
  cv::Mat mTcwGBA;

  // (cv::Mat) #TODO
  // 进行全局BA优化之前的当前关键帧的位姿.
  // 之所以要记录这个是因为在全局优化之后还要根据该关键帧在优化之前的位姿来更新地图点,which地图点的参考关键帧就是该关键帧
  cv::Mat mTcwBefGBA;

  // (long unsigned int) #TODO
  // 记录是由于哪个"当前关键帧"触发的全局BA,用来防止重复写入的事情发生(浪费时间)
  long unsigned int mnBAGlobalForKF;

  // (const float) #TODO
  // Calibration parameters
  const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

  // (const int) #TODO
  // Number of KeyPoints
  const int N;

  // (const std::vector<cv::KeyPoint>) #TODO
  // KeyPoints, stereo coordinate and descriptors (all associated by an index)
  // 和Frame类中的定义相同
  const std::vector<cv::KeyPoint> mvKeys;

  // (const std::vector<cv::KeyPoint>) #TODO
  const std::vector<cv::KeyPoint> mvKeysUn;

  // (const std::vector<float>) #TODO
  // negative value for monocular points
  const std::vector<float> mvuRight;

  // (const std::vector<float>) #TODO
  // negative value for monocular points
  const std::vector<float> mvDepth;

  // (const cv::Mat) #TODO
  const cv::Mat mDescriptors;

  // (DBoW2::BowVector) #TODO
  // BoW
  // Vector of words to represent images
  // mBowVec 内部实际存储的是std::map<WordId, WordValue>
  // WordId 和 WordValue 表示Word在叶子中的id 和权重
  DBoW2::BowVector mBowVec;

  // (DBoW2::FeatureVector) #TODO
  // Vector of nodes with indexes of local features
  // 内部实际存储 std::map<NodeId, std::vector<unsigned int> >
  // NodeId 表示节点id, std::vector<unsigned int>
  // 中实际存的是该节点id下所有特征点在图像中的索引
  DBoW2::FeatureVector mFeatVec;

  // (cv::Mat) #TODO
  // Pose relative to parent (this is computed when bad flag is activated)
  cv::Mat mTcp;

  // (const int) #TODO
  // Scale
  const int mnScaleLevels;

  // (const float) #TODO
  const float mfScaleFactor;

  // (const float) #TODO
  const float mfLogScaleFactor;

  // (const std::vector<float>) #TODO
  // 尺度因子, scale^n, scale=1.2, n为层数
  const std::vector<float> mvScaleFactors;

  // (const std::vector<float>) #TODO
  // 尺度因子的平方
  const std::vector<float> mvLevelSigma2;

  // (const std::vector<float>) #TODO
  const std::vector<float> mvInvLevelSigma2;

  // (const int) #TODO
  // Image bounds and calibration
  const int mnMinX;

  // (const int) #TODO
  const int mnMinY;

  // (const int) #TODO
  const int mnMaxX;

  // (const int) #TODO
  const int mnMaxY;

  // (const cv::Mat) #TODO
  const cv::Mat mK;

 protected:
  // (const cv::Mat) #TODO
  // SE3 Pose and camera center
  // The following variables need to be accessed trough a mutex to be thread
  // safe. ---- 但是大哥..protected也不是这样设计使用的啊
  // 当前相机的位姿, 世界坐标系到相机坐标系
  cv::Mat Tcw;

  // (const cv::Mat) #TODO
  // 当前相机位姿的逆
  cv::Mat Twc;

  // (const cv::Mat) #TODO
  // 相机光心(左目)在世界坐标系下的坐标,这里和普通帧中的定义是一样的
  cv::Mat Ow;

  // (const cv::Mat) #TODO
  // Stereo middel point. Only for visualization
  cv::Mat Cw;

  // (std::vector<MapPoint *>) #TODO
  // MapPoints associated to keypoints
  std::vector<MapPoint *> mvpMapPoints;

  // (KeyFrameDatabase *) #TODO
  // BoW
  KeyFrameDatabase *mpKeyFrameDB;

  // (ORBVocabulary *) #TODO
  // 词袋对象
  ORBVocabulary *mpORBvocabulary;

  // (std::vector<std::vector<std::vector<size_t>>>) #TODO
  // Grid over the image to speed up feature matching
  // ,其实应该说是二维的,第三维的 vector中保存的是这个网格内的特征点的索引
  std::vector<std::vector<std::vector<size_t>>> mGrid;

  // (std::map<KeyFrame *, int>) #TODO
  // Covisibility Graph
  // 与该关键帧连接(至少15个共视地图点)的关键帧与权重
  std::map<KeyFrame *, int> mConnectedKeyFrameWeights;

  // (std::vector<KeyFrame *>) #TODO
  // 共视关键帧中权重从大到小排序后的关键帧
  std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames;

  // (std::vector<int>) #TODO
  // 共视关键帧中从大到小排序后的权重, 和上面对应
  std::vector<int> mvOrderedWeights;

  // (bool) #TODO
  // Spanning Tree and Loop Edges
  // std::set是集合, 相比vector, 进行插入数据这样的操作时会自动排序
  // 是否是第一次生成树
  bool mbFirstConnection;

  // (KeyFrame *) #TODO
  KeyFrame *mpParent;  // 当前关键帧的父关键帧 (共视程度最高的)

  // (std::set<KeyFrame *>) #TODO
  // 存储当前关键帧的子关键帧
  std::set<KeyFrame *> mspChildrens;

  // (std::set<KeyFrame *>) #TODO
  // 和当前关键帧形成回环关系的关键帧
  std::set<KeyFrame *> mspLoopEdges;

  // (bool) #TODO
  // Bad flags
  // 当前关键帧已经和其他的关键帧形成了回环关系,
  // 因此在各种优化的过程中不应该被删除
  bool mbNotErase;

  // (bool) #TODO
  bool mbToBeErased;

  // (bool) #TODO
  bool mbBad;

  // (float) #TODO
  // 对于双目相机来说,双目相机基线长度的一半.
  // Only for visualization
  float mHalfBaseline;

  // (Map *) #TODO
  Map *mpMap;

  // (std::mutex) #TODO
  // 在对位姿进行操作时相关的互斥锁
  std::mutex mMutexPose;

  // (std::mutex) #TODO
  // 在操作当前关键帧和其他关键帧的公式关系的时候使用到的互斥锁
  std::mutex mMutexConnections;

  // (std::mutex) #TODO
  // 在操作和特征点有关的变量的时候的互斥锁
  std::mutex mMutexFeatures;
};

}  // namespace ORB_SLAM2

#endif  // INC_KEYFRAME_H_
