/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-12 22:27:13
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2, 单目初始化部分的声明. 双目
 * 和RGBD输入的情况下,不会使用这个类
 * @FilePath: /Localization/inc/Initializer.h
 */

#ifndef INC_INITIALIZER_H_
#define INC_INITIALIZER_H_

// 公用库.
#include <opencv2/opencv.hpp>

// ORB-SLAM2中的其他模块.
#include "Frame.h"

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
// 单目初始化数据类型.
// 单目SLAM初始化相关, 双目和RGBD不会使用这个类.
class Initializer {
 public:
  // 定义变量类型别名 Match,
  // 模板类 pair 中:
  // 第一个实例化参数是 #TODO
  // 第二个实例化参数是 #TODO
  typedef pair<int, int> Match;

 public:
  /**
   * @brief 根据参考帧构造初始化器
   * @note Fix the reference frame
   * @param ReferenceFrame: 参考帧
   * @param sigma: 测量误差
   * @param iterations: RANSAC迭代次数
   */
  Initializer(const Frame &ReferenceFrame, float sigma = 1.0,
              int iterations = 200);

  /**
   * @brief
   * 计算基础矩阵和单应性矩阵, 选取最佳的来恢复出最开始两帧之间的相对姿态,
   * 并进行三角化得到初始地图点 Step 1 重新记录特征点对的匹配关系 Step 2
   * 在所有匹配特征点对中随机选择8对匹配特征点为一组, 用于估计H矩阵和F矩阵 Step
   * 3 计算fundamental 矩阵 和homography 矩阵, 为了加速分别开了线程计算 Step 4
   * 计算得分比例来判断选取哪个模型来求位姿R,t
   * @note Computes in parallel a fundamental matrix and a homography
   * Selects a model and tries to recover the motion and the structure from
   * motion.
   * @param CurrentFrame: 当前帧, 也就是SLAM意义上的第二帧
   * @param vMatches12: 当前帧(2)和参考帧(1)图像中特征点的匹配关系
   * vMatches12[i]解释：i表示帧1中关键点的索引值,
   * vMatches12[i]的值为帧2的关键点索引值,
   * 没有匹配关系的话, vMatches12[i]值为-1
   * @param R21: 相机从参考帧到当前帧的旋转
   * @param t21: 相机从参考帧到当前帧的平移
   * @param vP3D: 三角化测量之后的三维地图点
   * @param vbTriangulated: 标记三角化点是否有效, 有效为true
   * @return (bool) true代表该帧可以成功初始化, 返回true;
   * false代表该帧不满足初始化条件, 返回false.
   */
  bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                  cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D,
                  vector<bool> &vbTriangulated);

 private:
  /**
   * @brief 计算单应矩阵
   * @note
   * 计算单应矩阵, 假设场景为平面情况下通过前两帧求取Homography矩阵,
   * 并得到该模型的评分 原理参考Multiple view geometry in computer vision  P109
   * 算法4.4 Step 1 将当前帧和参考帧中的特征点坐标进行归一化 Step 2
   * 选择8个归一化之后的点对进行迭代 Step 3 八点法计算单应矩阵矩阵 Step 4
   * 利用重投影误差为当次RANSAC的结果评分 Step 5
   * 更新具有最优评分的单应矩阵计算结果,并且保存所对应的特征点对的内点标记
   * @param vbMatchesInliers: 标记是否是外点
   * @param score: 计算单应矩阵的得分
   * @param H21: 单应矩阵结果
   * @return None
   */
  void FindHomography(vector<bool> &vbMatchesInliers, float &score,
                      cv::Mat &H21);

  /**
   * @brief 计算基础矩阵
   * @note
   * 计算基础矩阵, 假设场景为非平面情况下通过前两帧求取Fundamental矩阵,
   * 得到该模型的评分 Step 1 将当前帧和参考帧中的特征点坐标进行归一化 Step 2
   * 选择8个归一化之后的点对进行迭代 Step 3 八点法计算基础矩阵矩阵 Step 4
   * 利用重投影误差为当次RANSAC的结果评分 Step 5
   * 更新具有最优评分的基础矩阵计算结果,并且保存所对应的特征点对的内点标记
   * @param vbMatchesInliers: 标记是否是外点
   * @param score: 计算基础矩阵得分
   * @param F21: 基础矩阵结果
   * @return None
   */
  void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

  /**
   * @brief 用DLT方法求解单应矩阵H
   * @note
   * 这里最少用4对点就能够求出来, 不过这里为了统一还是使用了8对点求最小二乘解
   * @param vP1: 参考帧中归一化后的特征点
   * @param vP2: 当前帧中归一化后的特征点
   * @return (cv::Mat) 计算的单应矩阵
   */
  cv::Mat ComputeH21(const vector<cv::Point2f> &vP1,
                     const vector<cv::Point2f> &vP2);

  /**
   * @brief 根据特征点匹配求fundamental matrix(normalized 8点法)
   * @note
   * 注意F矩阵有秩为2的约束, 所以需要两次SVD分解
   * 参考： Multiple View Geometry in Computer Vision - Algorithm 11.1 p282
   * (中文版 p191)
   * @param vP1: 参考帧中归一化后的特征点
   * @param vP2: 当前帧中归一化后的特征点
   * @return (cv::Mat) 最后计算得到的基础矩阵F
   */
  cv::Mat ComputeF21(const vector<cv::Point2f> &vP1,
                     const vector<cv::Point2f> &vP2);

  /**
   * @brief 对给定的homography matrix打分,需要使用到卡方检验的知识
   * @note
   * @param H21: 从参考帧到当前帧的单应矩阵
   * @param H12: 从当前帧到参考帧的单应矩阵
   * @param vbMatchesInliers: 匹配好的特征点对的Inliers标记
   * @param sigma: 方差, 默认为1
   * @return (float) 返回得分
   */
  float CheckHomography(const cv::Mat &H21, const cv::Mat &H12,
                        vector<bool> &vbMatchesInliers, float sigma);

  /**
   * @brief 对给定的Fundamental matrix打分
   * @note
   * @param F21: 当前帧和参考帧之间的基础矩阵
   * @param vbMatchesInliers: 匹配的特征点对属于inliers的标记
   * @param sigma: 方差, 默认为1
   * @return (float) 返回得分
   */
  float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers,
                         float sigma);

  /**
   * @brief 从基础矩阵F中求解位姿R, t及三维点
   * @note
   * @param vbMatchesInliers: 匹配好的特征点对的Inliers标记
   * @param F21: 从参考帧到当前帧的基础矩阵
   * @param K: 相机的内参数矩阵
   * @param R21: 计算好的相机从参考帧到当前帧的旋转
   * @param t21: 计算好的相机从参考帧到当前帧的平移
   * @param vP3D: 三角化测量之后的特征点的空间坐标
   * @param vbTriangulated: 特征点三角化成功的标志
   * @param minParallax: 认为三角化有效的最小视差角
   * @param minTriangulated: 最小三角化点数量
   * @return (bool) true代表成功初始化; false代表初始化失败.
   */
  bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D,
                    vector<bool> &vbTriangulated, float minParallax,
                    int minTriangulated);

  /**
   * @brief 用H矩阵恢复R, t和三维点
   * @note
   * H矩阵分解常见有两种方法：Faugeras SVD-based decomposition 和 Zhang
   * SVD-based decomposition 代码使用了Faugeras SVD-based
   * decomposition算法, 参考文献 Motion and structure from motion in a piecewise
   * planar environment. International Journal of Pattern Recognition and
   * Artificial Intelligence, 1988
   * @param vbMatchesInliers: 匹配点对的内点标记
   * @param H21: 从参考帧到当前帧的单应矩阵
   * @param K: 相机的内参数矩阵
   * @param R21: 计算出来的相机旋转
   * @param t21: 计算出来的相机平移
   * @param vP3D: 世界坐标系下, 三角化测量特征点对之后得到的特征点的空间坐标
   * @param vbTriangulated: 特征点是否成功三角化的标记
   * @param minParallax: 对特征点的三角化测量中,
   * 认为其测量有效时需要满足的最小视差角(如果视差角过小则会引起非常大的观测误差),
   * 单位是角度
   * @param minTriangulated: 为了进行运动恢复,
   * 所需要的最少的三角化测量成功的点个数
   * @return (bool) true代表单应矩阵成功计算出位姿和三维点; false代表初始化失败.
   */
  bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D,
                    vector<bool> &vbTriangulated, float minParallax,
                    int minTriangulated);

  /**
   * @brief 给定投影矩阵P1, P2和图像上的匹配特征点点kp1, kp2, 从而计算三维点坐标
   * @note
   * @param kp1: 特征点, in reference frame
   * @param kp2: 特征点, in current frame
   * @param P1: 投影矩阵P1
   * @param P2: 投影矩阵P2
   * @param x3D: 计算的三维点
   * @return None
   */
  void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                   const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

  /**
   * @brief 归一化特征点到同一尺度, 作为后续normalize DLT的输入
   * @note
   *  [x' y' 1]' = T * [x y 1]'
   *  归一化后x', y'的均值为0, sum(abs(x_i'-0))=1, sum(abs((y_i'-0))=1
   *  为什么要归一化？
   *  在相似变换之后(点在不同的坐标系下),他们的单应性矩阵是不相同的
   *  如果图像存在噪声,使得点的坐标发生了变化,那么它的单应性矩阵也会发生变化
   *  我们采取的方法是将点的坐标放到同一坐标系下,并将缩放尺度也进行统一
   *  对同一幅图像的坐标进行相同的变换,不同图像进行不同变换
   *  缩放尺度是为了让噪声对于图像的影响在一个数量级上
   *  Step 1 计算特征点X,Y坐标的均值
   *  Step 2 计算特征点X,Y坐标离均值的平均偏离程度
   *  Step 3
   * 将x坐标和y坐标分别进行尺度归一化, 使得x坐标和y坐标的一阶绝对矩分别为1 Step
   * 4 计算归一化矩阵：其实就是前面做的操作用矩阵变换来表示而已
   * @param vKeys: 待归一化的特征点
   * @param vNormalizedPoints: 特征点归一化后的坐标
   * @param T: 归一化特征点的变换矩阵
   * @return None
   */
  void Normalize(const vector<cv::KeyPoint> &vKeys,
                 vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

  /**
   * @brief 进行cheirality check, 从而进一步找出F分解后最合适的解
   * @note
   * ReconstructF调用该函数进行cheirality
   * check, 从而进一步找出F分解后最合适的解
   * @param R: 待检查的相机旋转矩阵R
   * @param t: 待检查的相机旋转矩阵t
   * @param vKeys1: 参考帧特征点
   * @param vKeys2: 当前帧特征点
   * @param vMatches12: 两帧特征点的匹配关系
   * @param vbMatchesInliers: 特征点对的Inliers标记
   * @param K: 相机的内参数矩阵
   * @param vP3D: 三角化测量之后的特征点的空间坐标
   * @param th2: 重投影误差的阈值
   * @param vbGood: 特征点(对)中是good点的标记
   * @param parallax: 计算出来的比较大的视差角(注意不是最大,
   * 这个要看后面中程序的注释)
   * @return (int) 返回本组解中good点的数目
   */
  int CheckRT(const cv::Mat &R, const cv::Mat &t,
              const vector<cv::KeyPoint> &vKeys1,
              const vector<cv::KeyPoint> &vKeys2,
              const vector<Match> &vMatches12, vector<bool> &vbInliers,
              const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2,
              vector<bool> &vbGood, float &parallax);

  /**
   * @brief 分解Essential矩阵
   * @note
   * F矩阵通过结合内参可以得到Essential矩阵, 分解E矩阵将得到4组解
   * Multiple View Geometry in Computer Vision - Result 9.19 p259
   * 这4组解分别为[R1,t],[R1,-t],[R2,t],[R2,-t]
   * @param E: Essential Matrix
   * @param R1: Rotation Matrix 1
   * @param R2: Rotation Matrix 2
   * @param t: Translation, 另外一个结果取它的相反数就行
   * @return None
   */
  void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

  // (vector<cv::KeyPoint>) #TODO
  // Keypoints from Reference Frame (Frame 1)
  // 存储Reference Frame中的特征点
  vector<cv::KeyPoint> mvKeys1;

  // (vector<cv::KeyPoint>) #TODO
  // Keypoints from Current Frame (Frame 2)
  // 存储Current Frame中的特征点
  vector<cv::KeyPoint> mvKeys2;

  // (vector<Match>) #TODO
  // Current Matches from Reference to Current
  // Reference Frame: 1, Current Frame: 2
  // Match的数据结构是pair,mvMatches12只记录Reference到Current匹配上的特征点对
  vector<Match> mvMatches12;

  // (vector<bool>) #TODO
  // 记录Reference Frame的每个特征点在Current Frame是否有匹配的特征点
  vector<bool> mvbMatched1;

  // (cv::Mat) #TODO
  // Calibration
  // 相机内参
  cv::Mat mK;

  // (float) #TODO
  // Standard Deviation and Variance
  // 测量误差
  float mSigma;

  // (float) #TODO
  float mSigma2;

  // (int) #TODO
  // Ransac max iterations
  // 算Fundamental和Homography矩阵时RANSAC迭代次数
  int mMaxIterations;

  // (vector<vector<size_t>>) #TODO
  // Ransac sets
  // 二维容器, 外层容器的大小为迭代次数,
  // 内层容器大小为每次迭代算H或F矩阵需要的点,
  // 实际上是八对
  vector<vector<size_t>> mvSets;
};

}  // namespace ORB_SLAM2

#endif  // INC_INITIALIZER_H_
