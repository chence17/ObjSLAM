/*
 * @Author: Antonio Chan
 * @Date: 2021-04-07 22:00:25
 * @LastEditTime: 2021-04-14 13:16:08
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2,
 * 提供了一系列的常见转换.
 * orb中以cv::Mat为基本存储结构, 到g2o和Eigen需要一个转换.
 * 这些转换都很简单, 整个文件可以单独从orbslam里抽出来而不影响其他功能.
 * @FilePath: /Localization/inc/Converter.h
 */

#ifndef INC_CONVERTER_H_
#define INC_CONVERTER_H_

// 公用库.
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "third_party/g2o/g2o/types/types_seven_dof_expmap.h"
#include "third_party/g2o/g2o/types/types_six_dof_expmap.h"

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
// 数据转换数据类型.
// 这是一个完全的静态类, 没有成员变量, 所有的成员函数均为静态的.
class Converter {
 public:
  /**
   * @brief 描述子矩阵到单行的描述子向量的转换.
   * @note cv::Mat -> std::vector<cv::Mat>
   * 转换后的结果就是吧cv::Mat的每一行直接串联起来.
   * 应当注意, 这里的描述子矩阵是多个单行的cv::Mat组成的.
   * @param Descriptors: 待转换的描述子
   * @return (std::vector<cv::Mat>) 转换结果
   */
  static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

  /**
   * @brief 将以cv::Mat格式存储的位姿转换成为g2o::SE3Quat类型
   * @note v换成为g2o::SE3Quat类型
   * 将不同格式存储的位姿统一转换成为g2o::SE3Quat格式存储
   * @param cvT: #TODO
   * @return (g2o::SE3Quat) #TODO
   */
  static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

  /**
   * @brief 将以cv::Mat格式存储的位姿转换成为g2o::SE3Quat类型
   * @note v换成为g2o::SE3Quat类型
   * 将不同格式存储的位姿统一转换成为g2o::SE3Quat格式存储
   * @param gSim3: 以g2o::Sim3格式存储的位姿
   * @return (g2o::SE3Quat) #TODO
   */
  static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

  /**
   * @brief 将以g2o::SE3Quat格式存储的位姿转换成为cv::Mat格式
   * @note 将各种格式转换成为cv::Mat存储
   * @param SE3: 输入的g2o::SE3Quat格式存储的、待转换的位姿
   * @return (cv::Mat) 转换结果
   * @remark
   */
  static cv::Mat toCvMat(const g2o::SE3Quat &SE3);

  /**
   * @brief 将以g2o::Sim3格式存储的位姿转换成为cv::Mat格式
   * @note 将各种格式转换成为cv::Mat存储
   * @param Sim3: 输入的g2o::Sim3格式存储的、待转换的位姿
   * @return (cv::Mat) 转换结果
   * @remark
   */
  static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

  /**
   * @brief 将4x4 double型Eigen矩阵存储的位姿转换成为cv::Mat格式
   * @note 将各种格式转换成为cv::Mat存储
   * @param m: 输入Eigen矩阵
   * @return (cv::Mat) 转换结果
   * @remark
   */
  static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);

  /**
   * @brief 将一个3x1的Eigen行向量转换成为cv::Mat格式
   * @note 将各种格式转换成为cv::Mat存储
   * @param m: 3x1的Eigen行向量
   * @return (cv::Mat) 转换结果
   */
  static cv::Mat toCvMat(const Eigen::Matrix3d &m);

  /**
   * @brief 将一个3x1的Eigen行向量转换成为cv::Mat格式
   * @note 将各种格式转换成为cv::Mat存储
   * @param m: 3x1的Eigen行向量
   * @return (cv::Mat) 转换结果
   */
  static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);

  /**
   * @brief 将给定的旋转矩阵和平移向量转换为以cv::Mat存储的李群SE3
   * @note 其实就是组合旋转矩阵和平移向量来构造SE3
   * @param R: 旋转矩阵
   * @param t: 平移向量
   * @return (cv::Mat) 李群SE3
   */
  static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                         const Eigen::Matrix<double, 3, 1> &t);

  /**
   * @brief 将cv::Mat类型数据转换成为3x1的Eigen矩阵
   * @note 将给出的数据以Eigen库中对应的数据类型存储
   * 需要确保输入的数据大小尺寸正确.
   * @param cvVector: 待转换的数据
   * @return (Eigen::Matrix<double,3,1>) 转换结果
   */
  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);

  /**
   * @brief 将cv::Point3f转换成为Eigen中3x1的矩阵
   * @note 将给出的数据以Eigen库中对应的数据类型存储
   * @param cvPoint: 输入的cv表示的三维点坐标
   * @return (Eigen::Matrix<double,3,1>) 以Eigen中3x1向量表示的三维点坐标
   */
  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);

  /**
   * @brief 将一个3x3的cv::Mat矩阵转换成为Eigen中的矩阵
   * @note 将给出的数据以Eigen库中对应的数据类型存储
   * @param cvMat3: 输入
   * @return (Eigen::Matrix<double,3,3>) 转换结果
   */
  static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);

  /**
   * @brief
   * 将给定的cv::Mat类型的旋转矩阵转换成以std::vector<float>类型表示的四元数
   * @note 将给出的数据以Eigen库中对应的数据类型存储
   * 需要自己保证参数M满足旋转矩阵的定义
   * @param M: 以cv::Mat表示的旋转矩阵
   * @return (std::vector<float>) 四元数
   */
  static std::vector<float> toQuaternion(const cv::Mat &M);
};

}  // namespace ORB_SLAM2

#endif  // INC_CONVERTER_H_
