/*
 * @Author: Antonio Chan
 * @Date: 2021-04-07 22:00:25
 * @LastEditTime: 2021-04-13 11:36:10
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/Sim3Solver.h
 */

#ifndef INC_SIM3SOLVER_H_
#define INC_SIM3SOLVER_H_

// 公用库.
#include <opencv2/opencv.hpp>
#include <vector>

// ORB-SLAM2中的其他模块.
#include "KeyFrame.h"

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
// Sim3 求解器数据类型.
class Sim3Solver {
 public:
  /**
   * @brief Sim 3 Solver 构造函数
   * @note
   * @param pKF1: 当前关键帧
   * @param pKF2: 候选的闭环关键帧
   * @param vpMatched12: 通过词袋模型加速匹配所得到的,
   * 两帧特征点的匹配关系所得到的地图点,本质上是来自于候选闭环关键帧的地图点
   * @param bFixScale: 当前传感器类型的输入需不需要计算尺度.
   * 单目的时候需要, 双目和RGBD的时候就不需要了
   */
  Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2,
             const std::vector<MapPoint *> &vpMatched12,
             const bool bFixScale = true);

  /**
   * @brief 设置进行RANSAC时的参数
   * @note
   * @param probability: 当前这些匹配点的置信度,
   * 也就是一次采样恰好都是内点的概率
   * @param minInliers: 完成RANSAC所需要的最少内点个数
   * @param maxIterations: 设定的最大迭代次数
   * @return None
   */
  void SetRansacParameters(double probability = 0.99, int minInliers = 6,
                           int maxIterations = 300);

  /**
   * @brief 在下面的这个"进行迭代计算"函数的基础上套了一层壳,使用默认参数.
   * 不过目前好像没有被使用到
   * @note
   * @param vbInliers12: 内点标记,下标和构造时给出的地图点向量保持一致
   * @param nInliers: 内点数目
   * @return (cv::Mat) 计算得到的变换矩阵.
   * 如果期间计算出现了问题,那么返回的是空矩阵
   */
  cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

  /**
   * @brief Ransac求解mvX3Dc1和mvX3Dc2之间Sim3,
   * 函数返回mvX3Dc2到mvX3Dc1的Sim3变换
   * @note
   * @param nIterations: 设置的最大迭代次数
   * @param bNoMore: 为true表示穷尽迭代还没有找到好的结果, 说明求解失败
   * @param vbInliers: 标记是否是内点
   * @param nInliers: 内点数目
   * @return (cv::Mat) 计算得到的Sim3矩阵
   */
  cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers,
                  int &nInliers);

  /**
   * @brief 获取计算的旋转矩阵
   * @note
   * @return (cv::Mat) RANSAC过程中计算得到的最优解的旋转矩阵
   */
  cv::Mat GetEstimatedRotation();

  /**
   * @brief 获取计算的平移向量
   * @note
   * @return (cv::Mat) 平移向量
   */
  cv::Mat GetEstimatedTranslation();

  /**
   * @brief 获取估计的从候选帧到当前帧的变换尺度
   * @note
   * @return (float) 尺度因子
   */
  float GetEstimatedScale();

 protected:
  /**
   * @brief 给出三个点,计算它们的质心以及去质心之后的坐标
   * @note
   * @param P: 输入的3D点
   * @param Pr: 去质心后的点
   * @param C: 质心
   * @return None
   */
  void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

  /**
   * @brief 根据两组匹配的3D点,计算P2到P1的Sim3变换
   * @note
   * @param P1: 匹配的3D点(三个,每个的坐标都是列向量形式,
   * 三个点组成了3x3的矩阵)(当前关键帧)
   * @param P2: 匹配的3D点(闭环关键帧)
   * @return None
   */
  void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

  /**
   * @brief 通过计算的Sim3投影, 和自身投影的误差比较, 进行内点检测
   * @note
   * @return None
   */
  void CheckInliers();

  /**
   * @brief 按照给定的Sim3变换进行投影操作,得到三维点的2D投影点
   * @note
   * @param vP3Dw: 3D点
   * @param vP2D: 投影到图像的2D点
   * @param Tcw: Sim3变换
   * @param K: 内参
   * @return None
   */
  void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D,
               cv::Mat Tcw, cv::Mat K);

  /**
   * @brief 计算当前关键帧中的地图点在当前关键帧图像上的投影坐标
   * @note
   * @param vP3Dc: 相机坐标系下三维点坐标
   * @param vP2D: 投影的二维图像坐标
   * @param K: 内参矩阵
   * @return None
   */
  void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc,
                         std::vector<cv::Mat> &vP2D, cv::Mat K);

 protected:
  // (KeyFrame *) KeyFrames and matches
  // 当前关键帧
  KeyFrame *mpKF1;

  // (KeyFrame *) 闭环关键帧
  KeyFrame *mpKF2;

  // (std::vector<cv::Mat>) 存储匹配的,
  // 当前关键帧中的地图点在当前关键帧相机坐标系下的坐标
  std::vector<cv::Mat> mvX3Dc1;

  // (std::vector<cv::Mat>) 存储匹配的,
  // 闭环关键帧中的地图点在闭环关键帧相机坐标系下的坐标
  std::vector<cv::Mat> mvX3Dc2;

  // (std::vector<MapPoint *>) 匹配的地图点的中,存储当前关键帧的地图点
  std::vector<MapPoint *> mvpMapPoints1;

  // (std::vector<MapPoint *>) 匹配的地图点的中,存储闭环关键帧的地图点
  std::vector<MapPoint *> mvpMapPoints2;

  // (std::vector<MapPoint *>) 下标是当前关键帧中特征点的id,
  // 内容是对应匹配的, 闭环关键帧中的地图点
  std::vector<MapPoint *> mvpMatches12;

  // (std::vector<size_t>) 有效的匹配关系,在 vpMatched12 (构造函数)
  std::vector<size_t> mvnIndices1;

  // (std::vector<size_t>) 中的索引 这个变量好像是没有被用到
  std::vector<size_t> mvSigmaSquare1;

  // (std::vector<size_t>) 这个变量好像是没有被用到
  std::vector<size_t> mvSigmaSquare2;

  // (std::vector<size_t>) 当前关键帧中的某个特征点
  // 所允许的最大不确定度(和所在的金字塔图层有关)
  std::vector<size_t> mvnMaxError1;

  // (std::vector<size_t>) 闭环关键帧中的某个特征点所允许的最大不确定度(同上)
  std::vector<size_t> mvnMaxError2;

  // (int) 下面的这个匹配关系去掉坏点和非法值之后,得到的可靠的匹配关系的点的数目
  int N;

  // (int) 当前关键帧和闭环关键帧之间形成匹配关系的点的数目(Bow加速得到的匹配点)
  int mN1;

  // (cv::Mat) Current Estimation
  // 存储某次RANSAC过程中得到的旋转
  cv::Mat mR12i;

  // (cv::Mat) 存储某次RANSAC过程中得到的平移
  cv::Mat mt12i;

  // (float) 存储某次RANSAC过程中得到的缩放系数
  float ms12i;

  // (cv::Mat) 存储某次RANSAC过程中得到的变换矩阵
  cv::Mat mT12i;

  // (cv::Mat) 上面的逆
  cv::Mat mT21i;

  // (std::vector<bool>) 内点标记,
  // 下标和N,mvpMapPoints1等一致,用于记录某次迭代过程中的内点情况
  std::vector<bool> mvbInliersi;

  // (int) 在某次迭代的过程中经过投影误差进行的inlier检测得到的内点数目
  int mnInliersi;

  // (int) Current Ransac State
  // RANSAC迭代次数(当前正在进行的)
  int mnIterations;

  // (std::vector<bool>) 累计的,多次RANSAC中最好的最多的内点个数时的内点标记
  std::vector<bool> mvbBestInliers;

  // (int) 最好的一次迭代中,得到的内点个数
  int mnBestInliers;

  // (cv::Mat) 存储最好的一次迭代中得到的变换矩阵
  cv::Mat mBestT12;

  // (cv::Mat) 存储最好的一次迭代中得到的旋转
  cv::Mat mBestRotation;

  // (cv::Mat) 存储最好的一次迭代中得到的平移
  cv::Mat mBestTranslation;

  // (float) 存储最好的一次迭代中得到的缩放系数
  float mBestScale;

  // (bool) Scale is fixed to 1 in the stereo/RGBD case
  // 当前传感器输入的情况下,是否需要计算尺度
  bool mbFixScale;

  // (std::vector<size_t>) Indices for random selection
  // RANSAC中随机选择的时候,存储可以选择的点的id(去除那些存在问题的匹配点后重新排序)
  std::vector<size_t> mvAllIndices;

  // (std::vector<cv::Mat>) Projections
  // 当前关键帧中的地图点在当前关键帧图像上的投影坐标
  std::vector<cv::Mat> mvP1im1;

  // (std::vector<cv::Mat>) Projections
  // 闭环关键帧中的地图点在闭环关键帧图像上的投影坐标
  std::vector<cv::Mat> mvP2im2;

  // (double) RANSAC probability
  // 在计算RANSAC的理论迭代次数时使用到的概率,详细解释还是看函数
  // SetRansacParameters() 中的注释吧
  double mRansacProb;

  // (int) RANSAC min inliers
  // RANSAC 结束的理想条件:
  // 结束RANSAC过程所需要的最少内点数
  int mRansacMinInliers;

  // (int) RANSAC max iterations
  // RANSAC 结束的不理想条件: 最大迭代次数
  int mRansacMaxIts;

  // (float) Threshold inlier/outlier.
  // e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
  // 没有使用到的变量
  float mTh;

  // (float) #TODO
  // 没有使用到的变量
  float mSigma2;

  // (cv::Mat) Calibration
  // 当前关键帧的内参矩阵
  cv::Mat mK1;

  // (cv::Mat) Calibration
  // 闭环关键帧的内参矩阵
  cv::Mat mK2;
};

}  // namespace ORB_SLAM2

#endif  // INC_SIM3SOLVER_H_
