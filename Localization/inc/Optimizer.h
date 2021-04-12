/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-12 20:57:54
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/Optimizer.h
 */

#ifndef INC_OPTIMIZER_H_
#define INC_OPTIMIZER_H_

// 公用库.
#include <map>
#include <mutex>
#include <vector>

#include "third_party/g2o/g2o/types/types_seven_dof_expmap.h"

// ORB-SLAM2中的其他模块.
#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
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
// 需要使用到的其他模块的前置声明.
class LoopClosing;

// 优化器数据类型.
// 所有的优化相关的函数都在这个类中, 这个类只有成员函数没有成员变量,
// 相对要好分析一点.
class Optimizer {
 public:
  /**
   * @brief bundle adjustment Optimization
   * @note 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw)
   * 1. Vertex: g2o::VertexSE3Expmap(), 即当前帧的Tcw
   *            g2o::VertexSBAPointXYZ(), MapPoint的mWorldPos
   * 2. Edge:
   *     - g2o::EdgeSE3ProjectXYZ(), BaseBinaryEdge
   *         + Vertex：待优化当前帧的Tcw
   *         + Vertex：待优化MapPoint的mWorldPos
   *         + measurement：MapPoint在当前帧中的二维位置(u,v)
   *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
   * @param vpKFs: 关键帧
   * @param vpMP: MapPoints
   * @param nIterations: 迭代次数(20次)
   * @param pbStopFlag: 是否强制暂停
   * @param nLoopKF: 关键帧的个数 -- 但是我觉得形成了闭环关系的当前关键帧的id
   * @param bRobust: 是否使用核函数
   * @return None
   */
  void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF,
                               const std::vector<MapPoint *> &vpMP,
                               int nIterations = 5, bool *pbStopFlag = NULL,
                               const unsigned long nLoopKF = 0,
                               const bool bRobust = true);

  /**
   * @brief 进行全局BA优化, 但主要功能还是调用
   * @note BundleAdjustment,这个函数相当于加了一个壳.
   * @param pMap: 地图对象的指针
   * @param nIterations: 迭代次数
   * @param pbStopFlag: 外界给的控制GBA停止的标志位
   * @param nLoopKF: 当前回环关键帧的id, 其实也就是参与GBA的关键帧个数
   * @param bRobust: 是否使用鲁棒核函数
   * @return None
   */
  void static GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5,
                                     bool *pbStopFlag = NULL,
                                     const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);

  /**
   * @brief Local Bundle Adjustment
   * @note
   * 1. Vertex:
   *     - g2o::VertexSE3Expmap(), LocalKeyFrames,
   * 即当前关键帧的位姿、与当前关键帧相连的关键帧的位姿
   *     -
   * g2o::VertexSE3Expmap(), FixedCameras,
   * 即能观测到LocalMapPoints的关键帧(并且不属于LocalKeyFrames)的位姿,
   * 在优化中这些关键帧的位姿不变
   *     -
   * g2o::VertexSBAPointXYZ(), LocalMapPoints,
   * 即LocalKeyFrames能观测到的所有MapPoints的位置
   * 2. Edge:
   *     - g2o::EdgeSE3ProjectXYZ(), BaseBinaryEdge
   *         + Vertex：关键帧的Tcw, MapPoint的Pw
   *         + measurement：MapPoint在关键帧中的二维位置(u,v)
   *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
   *     - g2o::EdgeStereoSE3ProjectXYZ(), BaseBinaryEdge
   *         + Vertex：关键帧的Tcw, MapPoint的Pw
   *         + measurement：MapPoint在关键帧中的二维位置(ul,v,ur)
   *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
   * @param pKF: KeyFrame
   * @param pbStopFlag: 是否停止优化的标志
   * @param pMap: 在优化后, 更新状态时需要用到Map的互斥量mMutexMapUpdate
   * @note 由局部建图线程调用,对局部地图进行优化的函数
   * @return None
   */
  void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap);

  /**
   * @brief Pose Only Optimization
   * @note
   * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw)
   * 只优化Frame的Tcw, 不优化MapPoints的坐标
   * 1. Vertex: g2o::VertexSE3Expmap(), 即当前帧的Tcw
   * 2. Edge:
   *     - g2o::EdgeSE3ProjectXYZOnlyPose(), BaseUnaryEdge
   *         + Vertex：待优化当前帧的Tcw
   *         + measurement：MapPoint在当前帧中的二维位置(u,v)
   *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
   *     - g2o::EdgeStereoSE3ProjectXYZOnlyPose(), BaseUnaryEdge
   *         + Vertex：待优化当前帧的Tcw
   *         + measurement：MapPoint在当前帧中的二维位置(ul,v,ur)
   *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
   * @param: pFrame Frame
   * @return (int) inliers数量
   */
  int static PoseOptimization(Frame *pFrame);

  /**
   * @brief 闭环检测后, EssentialGraph优化
   * @note if bFixScale is true, 6DoF optimization (stereo,rgbd),
   * 7DoF otherwise (mono)
   * 1. Vertex:
   *     - g2o::VertexSim3Expmap, Essential graph中关键帧的位姿
   * 2. Edge:
   *     - g2o::EdgeSim3(), BaseBinaryEdge
   *         + Vertex：关键帧的Tcw, MapPoint的Pw
   *         + measurement：经过CorrectLoop函数步骤2, Sim3传播校正后的位姿
   *         + InfoMatrix: 单位矩阵
   * @param pMap: 全局地图
   * @param pLoopKF: 闭环匹配上的关键帧
   * @param pCurKF: 当前关键帧
   * @param NonCorrectedSim3: 未经过Sim3传播调整过的关键帧位姿
   * @param CorrectedSim3: 经过Sim3传播调整过的关键帧位姿
   * @param LoopConnections: 因闭环时MapPoints调整而新生成的边
   * @return None
   */
  void static OptimizeEssentialGraph(
      Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
      const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose &CorrectedSim3,
      const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
      const bool &bFixScale);

  /**
   * @brief 形成闭环时,对当前关键帧和闭环关键帧的Sim3位姿进行优化
   * @note if bFixScale is true, optimize SE3 (stereo,rgbd),
   * Sim3 otherwise (mono)
   * 闭环刚刚形成的时候,对当前关键帧和闭环关键帧之间的sim3变换的优化
   * 1. Vertex:
   *     - g2o::VertexSim3Expmap(), 两个关键帧的位姿
   *     - g2o::VertexSBAPointXYZ(), 两个关键帧共有的MapPoints
   * 2. Edge:
   *     - g2o::EdgeSim3ProjectXYZ(), BaseBinaryEdge
   *         + Vertex：关键帧的Sim3, MapPoint的Pw
   *         + measurement：MapPoint在关键帧中的二维位置(u,v)
   *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
   *     - g2o::EdgeInverseSim3ProjectXYZ(), BaseBinaryEdge
   *         + Vertex：关键帧的Sim3, MapPoint的Pw
   *         + measurement：MapPoint在关键帧中的二维位置(u,v)
   *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
   * @param pKF1: KeyFrame
   * @param pKF2: KeyFrame
   * @param vpMatches1: 两个关键帧的匹配关系
   * @param g2oS12: 两个关键帧间的Sim3变换
   * @param th2: 核函数阈值
   * @param bFixScale: 是否优化尺度, 弹目进行尺度优化, 双目不进行尺度优化
   * @return (int) #TODO
   */
  static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2,
                          std::vector<MapPoint *> &vpMatches1,
                          g2o::Sim3 &g2oS12, const float th2,
                          const bool bFixScale);
};

}  // namespace ORB_SLAM2

#endif  // INC_OPTIMIZER_H_
