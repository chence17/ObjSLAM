/*
 * @Author: Antonio Chan
 * @Date: 2021-04-04 21:27:51
 * @LastEditTime: 2021-04-04 21:34:43
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/Converter.h
 */

#ifndef INC_CONVERTER_H_
#define INC_CONVERTER_H_

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "g2o/types/types_seven_dof_expmap.h"
#include "g2o/types/types_six_dof_expmap.h"

namespace ORB_SLAM2 {

class Converter {
 public:
  static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

  static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
  static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

  static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
  static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
  static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);
  static cv::Mat toCvMat(const Eigen::Matrix3d &m);
  static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);
  static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                         const Eigen::Matrix<double, 3, 1> &t);

  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);
  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);
  static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);

  static std::vector<float> toQuaternion(const cv::Mat &M);
};

}  // namespace ORB_SLAM2

#endif  // INC_CONVERTER_H_
