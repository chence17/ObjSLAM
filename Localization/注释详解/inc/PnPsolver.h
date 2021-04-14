/*
 * @Author: Antonio Chan
 * @Date: 2021-04-07 22:00:25
 * @LastEditTime: 2021-04-14 14:45:31
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/PnPsolver.h
 */

#ifndef INC_PNPSOLVER_H_
#define INC_PNPSOLVER_H_

// 公用库.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>

#include "third_party/DBoW2/DUtils/Random.h"

// ORB-SLAM2中的其他模块.
#include "Frame.h"
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
// 相机位姿求解器数据类型.
class PnPsolver {
 public:
  /**
   * @brief 构造函数
   * @note
   * @param F: 当前要求解位姿的帧
   * @param vpMapPointMatches: 地图点, 可能来自与局部地图,
   * 也可能来自于参考关键帧
   */
  PnPsolver(const Frame &F, const vector<MapPoint *> &vpMapPointMatches);

  /**
   * @brief 析构函数
   * @note
   */
  ~PnPsolver();

  /**
   * @brief 设置RANSAC迭代的参数
   * @note
   * @param probability: 用于计算RANSAC理论迭代次数所用的概率
   * @param minInliers: 退出RANSAC所需要的最小内点个数,
   * 注意这个只是给定值,最终迭代的时候不一定按照这个来
   * @param maxIterations: 设定的最大RANSAC迭代次数
   * @param minSet: 表示求解这个问题所需要的最小的样本数目,
   * 简称最小集;参与到最小内点数的确定过程中
   * @param epsilon: 希望得到的 内点数/总体数 的比值,
   * 参与到最小内点数的确定过程中
   * @param th2: 内外点判定时的距离的baseline
   * (程序中还会根据特征点所在的图层对这个阈值进行缩放的)
   * @return None
   */
  void SetRansacParameters(double probability = 0.99, int minInliers = 8,
                           int maxIterations = 300, int minSet = 4,
                           float epsilon = 0.4, float th2 = 5.991);

  /**
   * @brief brief
   * @note REVIEW 目测这个函数没有被调用过
   * @param vbInliers: #TODO
   * @param nInliers: #TODO
   * @return (cv::Mat) #TODO
   */
  cv::Mat find(vector<bool> &vbInliers, int &nInliers);

  /**
   * @brief 进行迭代计算
   * @note
   * @param nIterations: 给定的迭代次数,但是程序也有可能不听这个
   * @param bNoMore: 已经达到了最大迭代次数的标志
   * @param vbInliers: 内点标记
   * @param nInliers: 内点数目
   * @return (cv::Mat) Tcw
   */
  cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers,
                  int &nInliers);

 private:
  /**
   * @brief 通过之前求解的位姿来进行3D-2D投影, 统计内点数目
   * @note
   * @return None
   */
  void CheckInliers();

  /**
   * @brief 使用已经是内点的匹配点对, 再进行一次EPnP过程, 进行相机位姿的求精.
   * @note
   * @return (bool)
   * 返回的结果表示经过求精过程后的内点数,能否达到退出RANSAC的要求
   */
  bool Refine();

  /**==============================================
   * *                   注意
   *   Functions from the original EPnP code
   *
   *=============================================**/

  /**
   * @brief 更新原始EPnP代码中使用的"最小集",
   * 如果符合更新条件的话还是重新生成一些用于计算的数组(不安全类型的那种,所以无法直接缩小)
   * @note
   * @param n: 最小集
   * @return None
   */
  void set_maximum_number_of_correspondences(const int n);

  /**
   * @brief 清空当前已有的匹配点计数,为进行新的一次迭代作准备
   * @note
   * @return None
   */
  void reset_correspondences(void);

  /**
   * @brief EPnP部分的函数,向堆内存的数组中添加匹配点对
   * @note
   * @param X: 3D点
   * @param Y: 3D点
   * @param Z: 3D点
   * @param u: 2D点
   * @param v: 2D点
   * @return None
   */
  void add_correspondence(const double X, const double Y, const double Z,
                          const double u, const double v);

  /**
   * @brief 使用EPnP算法计算相机的位姿.其中匹配点的信息由类的成员函数给定
   * @note
   * @param R: 旋转
   * @param T: 平移
   * @return (double) 使用这对旋转和平移的时候, 匹配点对的平均重投影误差
   */
  double compute_pose(double R[3][3], double T[3]);

  /**
   * @brief 目测没有使用到的函数,
   * 在原版的EPnP中用于计算计算值和真值之间的相对误差
   * @note 在ORB中没有用到
   * @param rot_err: 计算得到的相对旋转误差
   * @param transl_err: 计算得到的相对平移误差
   * @param Rtrue: 旋转真值
   * @param ttrue: 平移真值
   * @param Rest: 旋转计算值
   * @param test: 平移计算值
   * @return None
   */
  void relative_error(double &rot_err, double &transl_err,
                      const double Rtrue[3][3], const double ttrue[3],
                      const double Rest[3][3], const double test[3]);

  /**
   * @brief 输出计算得到的位姿.
   * 这个函数是EPnP源码中自带的, 用于调试,
   * 所以在ORB中应当算是一个被放弃了的函数(因为我们并不需要输出啊)
   * @note 在ORB中没有用到
   * @param R: 旋转
   * @param t: 平移
   * @return None
   */
  void print_pose(const double R[3][3], const double t[3]);

  /**
   * @brief 计算在给定位姿的时候的3D点投影误差
   * @note
   * @param R: 给定旋转
   * @param t: 给定平移
   * @return (double) 重投影误差,是平均到每一对匹配点上的误差
   */
  double reprojection_error(const double R[3][3], const double t[3]);

  /**
   * @brief 从给定的匹配点中计算出四个控制点
   * @note
   * @return None
   */
  void choose_control_points(void);

  /**
   * @brief 求解世界坐标系下四个控制点的系数alphas, 在相机坐标系下系数不变
   * @note
   * @return None
   */
  void compute_barycentric_coordinates(void);

  /**
   * @brief 根据提供的每一对点的数据来填充Moment Matrix M.
   * 每对匹配点的数据可以填充两行
   * @note
   * @param M: cvMat对应,存储矩阵M
   * @param row: 开始填充数据的行
   * @param alphas: 3D点,为这个空间点在当前控制点坐标系下的表示(a1~a4)
   * @param u: 2D点坐标
   * @param v: 2D点坐标
   * @return None
   */
  void fill_M(CvMat *M, const int row, const double *alphas, const double u,
              const double v);

  /**
   * @brief 通过给出的beta和vi,计算控制点在相机坐标系下的坐标
   * @note
   * @param betas: beta
   * @param ut: 其实是vi
   * @return None
   */
  void compute_ccs(const double *betas, const double *ut);

  /**
   * @brief 根据相机坐标系下控制点坐标ccs 和控制点系数
   * alphas( 通过世界坐标系下3D点计算得到) , 得到相机坐标系下3D点坐标 pcs
   * 过程可以参考 https://blog.csdn.net/jessecw79/article/details/82945918
   * @note
   * @return None
   */
  void compute_pcs(void);

  /**
   * @brief 保持所有点在相机坐标系下的深度为正,调整符号
   * @note
   * @return None
   */
  void solve_for_sign(void);

  /**
   * @brief 计算N=4时候的粗糙近似解, 暴力将其他量置为0
   * @note
   * @param L_6x10: 矩阵L
   * @param Rho: 非齐次项 \rho, 列向量
   * @param betas: 计算得到的beta
   * @return None
   */
  void find_betas_approx_1(const CvMat *L_6x10, const CvMat *Rho,
                           double *betas);

  /**
   * @brief 计算N=2时候的粗糙近似解, 暴力将其他量置为0
   * @note
   * @param L_6x10: 矩阵L
   * @param Rho: 非齐次项 \rho, 列向量
   * @param betas: 计算得到的beta
   * @return None
   */
  void find_betas_approx_2(const CvMat *L_6x10, const CvMat *Rho,
                           double *betas);

  /**
   * @brief 计算N=3时候的粗糙近似解, 暴力将其他量置为0
   * @note
   * @param L_6x10: 矩阵L
   * @param Rho: 非齐次项 \rho, 列向量
   * @param betas: 计算得到的beta
   * @return None
   */
  void find_betas_approx_3(const CvMat *L_6x10, const CvMat *Rho,
                           double *betas);

  /**
   * @brief 使用QR分解来求解增量方程
   * @note
   * @param A: 洗漱矩阵
   * @param b: 非齐次项
   * @param X: 增量
   * @return None
   */
  void qr_solve(CvMat *A, CvMat *b, CvMat *X);

  /**
   * @brief 计算两个三维向量的点乘
   * @note
   * @param v1: 向量1
   * @param v2: 向量2
   * @return (double) 计算结果
   */
  double dot(const double *v1, const double *v2);

  /**
   * @brief 计算两个三维向量所表示的空间点的欧式距离的平方
   * @note
   * @param p1: 点1
   * @param p2: 点2
   * @return (double) 计算的距离结果
   */
  double dist2(const double *p1, const double *p2);

  /**
   * @brief 计算四个控制点任意两点间的距离, 总共6个距离,
   * 对应论文式13中的向量\rho
   * @note
   * @param rho: 计算结果
   * @return None
   */
  void compute_rho(double *rho);

  /**
   * @brief 计算矩阵L,论文式13中的L矩阵,不过这里的是按照N=4的时候计算的
   * @note
   * @param ut: 特征值分解之后得到的12x12特征矩阵
   * @param l_6x10: 计算的L矩阵结果, 维度6x10
   * @return None
   */
  void compute_L_6x10(const double *ut, double *l_6x10);

  /**
   * @brief 对计算出来的Beta结果进行高斯牛顿法优化,求精.
   * @note
   * 过程参考EPnP论文中式(15)
   * @param L_6x10: L矩阵
   * @param Rho: Rho向量
   * @param current_betas: 优化之后的Beta
   * @return None
   */
  void gauss_newton(const CvMat *L_6x10, const CvMat *Rho,
                    double current_betas[4]);

  /**
   * @brief 计算高斯牛顿法优化时,增量方程中的系数矩阵和非齐次项
   * @note
   * @param l_6x10: L矩阵
   * @param rho: Rho矩向量
   * @param cb: 当前次迭代得到的beta1~beta4
   * @param A: 计算得到的增量方程中的系数矩阵
   * @param b: 计算得到的增量方程中的非齐次项
   * @return None
   */
  void compute_A_and_b_gauss_newton(const double *l_6x10, const double *rho,
                                    double cb[4], CvMat *A, CvMat *b);

  /**
   * @brief 根据已经得到的控制点在当前相机坐标系下的坐标来恢复出相机的位姿
   * @note
   * @param ut: vi
   * @param betas: betas
   * @param R: 计算得到的相机旋转R
   * @param t: 计算得到的相机位置t
   * @return (double) 使用这个位姿,所得到的重投影误差
   */
  double compute_R_and_t(const double *ut, const double *betas, double R[3][3],
                         double t[3]);

  /**
   * @brief 用3D点在世界坐标系和相机坐标系下对应的坐标, 用ICP求取R t
   * @note
   * @param R: 旋转
   * @param t: 平移
   * @return None
   */
  void estimate_R_and_t(double R[3][3], double t[3]);

  /**
   * @brief 复制计算得到的位姿到另外的一组变量中
   * @note
   * @param R_dst: #TODO
   * @param t_dst: #TODO
   * @param R_src: #TODO
   * @param t_src: #TODO
   * @return None
   */
  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
                    double R_src[3][3], double t_src[3]);

  /**
   * @brief 目测没有使用到的函数,
   * 在原版的EPnP中勇于将旋转矩阵转换成为四元数的表达形式
   * @note 在ORB中没有用到
   * @param R: 需要转换的旋转矩阵
   * @param q: 转换后得到的四元数表达
   * @return None
   */
  void mat_to_quat(const double R[3][3], double q[4]);

 private:
  // (double) 相机内参
  double uc, vc, fu, fv;

  // (double*) 3D点在世界坐标系下在坐标
  // 组织形式: x1 y1 z1 | x2 y2 z2 | ...
  double *pws;

  // (double*) 图像坐标系下的2D点坐标
  // 组织形式: u1 v1 | u2 v2 | ...
  double *us;

  // (double*) 真实3D点用4个虚拟控制点表达时的系数
  // 组织形式: a11 a12 a13 a14 | a21 a22 a23 a24 | ...
  // 每个匹配点都有自己的a1~a4
  double *alphas;

  // (double*) 3D点在当前帧相机坐标系下的坐标
  double *pcs;

  // (int) 每次RANSAC计算的过程中使用的匹配点对数的最大值,
  // 其实应该和最小集的大小是完全相同的
  int maximum_number_of_correspondences;

  // (int) 当前次迭代中,已经采样的匹配点的个数,
  // 默认值为4
  int number_of_correspondences;

  // (double**) 存储控制点在世界坐标系下的坐标, 第一维表示是哪个控制点,
  // 第二维表示是哪个坐标(x,y,z)
  double cws[4][3];

  // (double**) 存储控制点在相机坐标系下的坐标, 含义同上
  double ccs[4][3];

  // (double) 没有被使用到的变量, 但是看变量名字,
  // 应该是用于存储某个矩阵的行列式值的
  double cws_determinant;

  // (vector<MapPoint *>) 存储构造的时候给出的地图点
  vector<MapPoint *> mvpMapPointMatches;

  // (vector<cv::Point2f>) 2D Points
  // 存储当前帧的2D点,由特征点转换而来,只保存了坐标信息
  vector<cv::Point2f> mvP2D;

  // (vector<float>) 和2D特征点向量下标对应的尺度和不确定性信息
  // (从该特征点所在的金字塔图层有关)
  vector<float> mvSigma2;

  // (vector<cv::Point3f>) 3D Points
  // 存储给出的地图点中有效的地图点(在世界坐标系下的坐标)
  vector<cv::Point3f> mvP3Dw;

  // (vector<size_t>) 记录构造时给出的地图点对应在帧中的特征点的id,
  // 这个是"跳跃的" Index in Frame
  vector<size_t> mvKeyPointIndices;

  // (double) Current Estimation
  // 在某次RANSAC迭代过程中计算得到的旋转矩阵
  double mRi[3][3];

  // (double) 在某次RANSAC迭代过程中计算得到的平移向量
  double mti[3];

  // (cv::Mat) 在程序中并没有被使用到的变量
  cv::Mat mTcwi;

  // (vector<bool>) 记录每次迭代时的inlier点
  vector<bool> mvbInliersi;

  // (int) 记录每次迭代时的inlier点的数目
  int mnInliersi;

  // (int) Current Ransac State
  // 历史上已经进行的RANSAC迭代次数
  int mnIterations;

  // (vector<bool>) 历史上最好一次迭代时的内点标记
  vector<bool> mvbBestInliers;

  // (int) 历史上的迭代中最多的内点数
  int mnBestInliers;

  // (cv::Mat) 历史上最佳的一次迭代所计算出来的相机位姿
  cv::Mat mBestTcw;

  // (cv::Mat) Refined
  // 求精之后得到的相机位姿
  cv::Mat mRefinedTcw;

  // (vector<bool>) 求精之后的内点标记
  vector<bool> mvbRefinedInliers;

  // (int) 求精之后的内点数
  int mnRefinedInliers;

  // (int) Number of Correspondences
  // 就是 mvP2D 的大小, 表示给出帧中和地图点匹配的特征点的个数,
  // 也就是匹配的对数(相当于采样的总体)
  int N;

  // (vector<size_t>) Indices for random selection [0 .. N-1]
  // 记录特征点在当前求解器中的向量中存储的索引, 是连续的
  // 存储了供RANSAC过程使用的点的下标
  vector<size_t> mvAllIndices;

  // (double) RANSAC probability
  // 计算RANSAC迭代次数的理论值的时候用到的概率,和Sim3Slover中的一样
  double mRansacProb;

  // (int) RANSAC min inliers
  // 正常退出RANSAC的时候需要达到的最最少的内点个数f
  int mRansacMinInliers;

  // (int) RANSAC max iterations
  // RANSAC的最大迭代次数
  int mRansacMaxIts;

  // (float) RANSAC expected inliers/total ratio
  // RANSAC中,最小内点数占全部点个数的比例
  float mRansacEpsilon;

  // (float) RANSAC Threshold inlier/outlier.
  // Max error e = dist(P1,T_12*P2)^2
  // 在程序中并没有使用到的变量
  float mRansacTh;

  // (int) RANSAC Minimun Set used at each iteration
  // 为每次RANSAC需要的特征点数, 默认为4组3D-2D对应点.
  // 参与到最少内点数的确定过程中
  int mRansacMinSet;

  // (vector<float>) Max square error associated with scale level.
  // Max error = th*th*sigma(level)*sigma(level)
  // 存储不同图层上的特征点在进行内点验证的时候,使用的不同的距离阈值
  vector<float> mvMaxError;
};

}  // namespace ORB_SLAM2

#endif  // INC_PNPSOLVER_H_
