/*
 * @Author: Antonio Chan
 * @Date: 2021-04-07 22:00:25
 * @LastEditTime: 2021-04-14 14:46:22
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * 这个文件主要负责进行ORB特征点的提取和数目分配功能
 * @FilePath: /Localization/inc/ORBextractor.h
 */

#ifndef INC_ORBEXTRACTOR_H_
#define INC_ORBEXTRACTOR_H_

// 公用库.
#include <opencv/cv.h>

#include <iostream>
#include <iterator>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

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
// 提取器节点数据类型.
// 用于在特征点的分配过程中,
// 分配四叉树时用到的结点数据类型.
class ExtractorNode {
 public:
  /**
   * @brief 构造函数
   * @note
   */
  ExtractorNode() : bNoMore(false) {}

  /**
   * @brief 在八叉树分配特征点的过程中, 实现一个节点分裂为4个节点的操作
   * @note
   * @param n1: 分裂的节点1
   * @param n2: 分裂的节点2
   * @param n3: 分裂的节点3
   * @param n4: 分裂的节点4
   * @return None
   */
  void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3,
                  ExtractorNode &n4);

 public:
  // (std::vector<cv::KeyPoint>) 保存有当前节点的特征点
  std::vector<cv::KeyPoint> vKeys;

  // (cv::Point2i) 当前节点所对应的图像坐标边界
  cv::Point2i UL, UR, BL, BR;

  // (std::list<ExtractorNode>::iterator) 存储提取器节点的列表
  // (其实就是双向链表)的一个迭代器,可以参考
  // [http://www.runoob.com/cplusplus/cpp-overloading.html]
  // 这个迭代器提供了访问总节点列表的方式, 需要结合cpp文件进行分析
  std::list<ExtractorNode>::iterator lit;

  // (bool) 如果节点中只有一个特征点的话, 说明这个节点不能够再进行分裂了,
  // 这个标志置位 这个节点中如果没有特征点的话, 这个节点就直接被删除了
  bool bNoMore;
};

// ORB特征点提取器数据类型.
class ORBextractor {
 public:
  // TODO 但是在程序中好像并没有被用到
  // 定义一个枚举类型用于表示使用HARRIS响应值还是使用FAST响应值
  enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

  /**
   * @brief 构造函数
   * @note
   * 之所以会有两种响应值的阈值, 原因是,
   * 程序先使用初始的默认FAST响应值阈值提取图像cell中的特征点；如果提取到的
   * 特征点数目不足, 那么就降低要求, 使用较小FAST响应值阈值进行再次提取,
   * 以获得尽可能多的FAST角点.
   * @param nfeatures: 指定要提取出来的特征点数目
   * @param scaleFactor: 图像金字塔的缩放系数
   * @param nlevels: 指定需要提取特征点的图像金字塔层
   * @param iniThFAST: 初始的默认FAST响应值阈值
   * @param minThFAST: 较小的FAST响应值阈值
   */
  ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST,
               int minThFAST);

  /**
   * @brief 析构函数
   * @note
   */
  ~ORBextractor() {}

  /**
   * @brief 使用八叉树的方法将提取到的ORB特征点尽可能均匀地分布在整个图像中
   * @note
   * Compute the ORB features and descriptors on an image.
   * ORB are dispersed on the image using an octree.
   * Mask is ignored in the current implementation.
   * 的确是, 函数中实际上并没有用到MASK.
   * 这里是重载了这个ORBextractor类的括号运算符;函数中实际上并没有用到MASK这个参数.
   * @param image: 要操作的图像
   * @param mask: 图像掩膜, 辅助进行图片处理,
   * 可以参考[https://www.cnblogs.com/skyfsm/p/6894685.html]
   * @param keypoints: 保存提取出来的特征点的向量
   * @param descriptors: 输出用的保存特征点描述子的cv::Mat
   * @return None
   */
  void operator()(cv::InputArray image, cv::InputArray mask,
                  std::vector<cv::KeyPoint> &keypoints,
                  cv::OutputArray descriptors);

  /**==============================================
   * *                   内联函数
   *   下面的这些内联函数都是用来直接获取类的成员变量的
   *
   *=============================================**/

  /**
   * @brief 获取图像金字塔的层数
   * @note
   * @return (int) 图像金字塔的层数
   */
  int inline GetLevels() { return nlevels; }

  /**
   * @brief 获取当前提取器所在的图像的缩放因子.
   * @note 这个不带s的因子表示是相临近层之间的.
   * @return (float) 当前提取器所在的图像的缩放因子, 相邻层之间
   */
  float inline GetScaleFactor() { return scaleFactor; }

  /**
   * @brief 获取图像金字塔中每个图层相对于底层图像的缩放因子
   * @note
   * @return (std::vector<float>) 图像金字塔中每个图层相对于底层图像的缩放因子
   */
  std::vector<float> inline GetScaleFactors() { return mvScaleFactor; }

  /**
   * @brief 获取上面的那个缩放因子s的倒数
   * @note
   * @return (std::vector<float>) 倒数
   */
  std::vector<float> inline GetInverseScaleFactors() {
    return mvInvScaleFactor;
  }

  /**
   * @brief 获取sigma^2, 就是每层图像相对于初始图像缩放因子的平方,
   * 参考cpp文件中类构造函数的操作
   * @note
   * @return (std::vector<float>) sigma^2
   */
  std::vector<float> inline GetScaleSigmaSquares() { return mvLevelSigma2; }

  /**
   * @brief 获取上面sigma平方的倒数
   * @note
   * @return (std::vector<float>) #TODO
   */
  std::vector<float> inline GetInverseScaleSigmaSquares() {
    return mvInvLevelSigma2;
  }

 public:
  // (std::vector<cv::Mat>) 这个是用来存储图像金字塔的变量, 一个元素存储一层图像
  std::vector<cv::Mat> mvImagePyramid;

 protected:
  /**
   * @brief 针对给出的一张图像, 计算其图像金字塔
   * @note
   * @param image: 给出的图像
   * @return None
   */
  void ComputePyramid(cv::Mat image);

  /**
   * @brief 以八叉树分配特征点的方式, 计算图像金字塔中的特征点
   * @note 这里两层vector的意思是, 第一层存储的是某张图片中的所有特征点,
   * 而第二层则是存储图像金字塔中所有图像的vectors of keypoints
   * @param allKeypoints: 提取得到的所有特征点
   * @return None
   */
  void ComputeKeyPointsOctTree(
      std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

  /**
   * @brief 对于某一图层, 分配其特征点, 通过八叉树的方式
   * @note
   * @param vToDistributeKeys: 等待分配的特征点
   * @param minX: 分发的图像范围
   * @param maxX: 分发的图像范围
   * @param minY: 分发的图像范围
   * @param maxY: 分发的图像范围
   * @param nFeatures: 设定的、本图层中想要提取的特征点数目
   * @param level: 要提取的图像所在的金字塔层
   * @return (std::vector<cv::KeyPoint>) #TODO
   */
  std::vector<cv::KeyPoint> DistributeOctTree(
      const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
      const int &maxX, const int &minY, const int &maxY, const int &nFeatures,
      const int &level);

  /**
   * @brief 这是使用另外一种老办法提取并平均特征点的方法,
   * 但是在实际的程序中并没有用到
   * @note
   * @param allKeypoints: 提取到的特征点
   * @return None
   */
  void ComputeKeyPointsOld(
      std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

 protected:
  /**==============================================
   * *                   疑惑
   *   作者不地道啊, 这里是类的成员变量, 说好的变量名的m前缀呢？
   *
   *=============================================**/

  // (std::vector<cv::Point>) 用于计算描述子的随机采样点集合
  std::vector<cv::Point> pattern;

  // (int) 整个图像金字塔中, 要提取的特征点数目
  int nfeatures;

  // (double) 图像金字塔层与层之间的缩放因子
  double scaleFactor;

  // (int) 图像金字塔的层数
  int nlevels;

  // (int) 初始的FAST响应值阈值
  int iniThFAST;

  // (int) 最小的FAST响应值阈值
  int minThFAST;

  // (std::vector<int>) 分配到每层图像中, 要提取的特征点数目
  std::vector<int> mnFeaturesPerLevel;

  // (std::vector<int>) 计算特征点方向的时候, 有个圆形的图像区域,
  // 这个vector中存储了每行u轴的边界(四分之一, 其他部分通过对称获得)
  std::vector<int> umax;

  // (std::vector<float>) 每层图像的缩放因子
  std::vector<float> mvScaleFactor;

  // (std::vector<float>) 以及每层缩放因子的倒数
  std::vector<float> mvInvScaleFactor;

  // (std::vector<float>) 存储每层的sigma^2,
  // 即上面每层图像相对于底层图像缩放倍数的平方
  std::vector<float> mvLevelSigma2;

  // (std::vector<float>) sigma平方的倒数
  std::vector<float> mvInvLevelSigma2;
};

}  // namespace ORB_SLAM2

#endif  // INC_ORBEXTRACTOR_H_
