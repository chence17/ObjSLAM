/*
 * @Author: Antonio Chan
 * @Date: 2021-04-07 22:00:25
 * @LastEditTime: 2021-04-14 14:55:08
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/Examples/ROS/ORB_SLAM2/src/AR/ViewerAR.h
 */

#ifndef VIEWERAR_H
#define VIEWERAR_H

#include <pangolin/pangolin.h>

#include <mutex>
#include <opencv2/core/core.hpp>
#include <string>

#include "../../../include/System.h"

namespace ORB_SLAM2 {

class Plane {
 public:
  Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw);
  Plane(const float &nx, const float &ny, const float &nz, const float &ox,
        const float &oy, const float &oz);

  void Recompute();

  // normal
  cv::Mat n;
  // origin
  cv::Mat o;
  // arbitrary orientation along normal
  float rang;
  // transformation from world to the plane
  cv::Mat Tpw;
  pangolin::OpenGlMatrix glTpw;
  // MapPoints that define the plane
  std::vector<MapPoint *> mvMPs;
  // camera pose when the plane was first observed (to compute normal direction)
  cv::Mat mTcw, XC;
};

class ViewerAR {
 public:
  ViewerAR();

  void SetFPS(const float fps) {
    mFPS = fps;
    mT = 1e3 / fps;
  }

  void SetSLAM(ORB_SLAM2::System *pSystem) { mpSystem = pSystem; }

  // Main thread function.
  void Run();

  void SetCameraCalibration(const float &fx_, const float &fy_,
                            const float &cx_, const float &cy_) {
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
  }

  void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status,
                    const std::vector<cv::KeyPoint> &vKeys,
                    const std::vector<MapPoint *> &vMPs);

  void GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status,
                    std::vector<cv::KeyPoint> &vKeys,
                    std::vector<MapPoint *> &vMPs);

 private:
  // SLAM
  ORB_SLAM2::System *mpSystem;

  void PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im);
  void AddTextToImage(const std::string &s, cv::Mat &im, const int r = 0,
                      const int g = 0, const int b = 0);
  void LoadCameraPose(const cv::Mat &Tcw);
  void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im);
  void DrawCube(const float &size, const float x = 0, const float y = 0,
                const float z = 0);
  void DrawPlane(int ndivs, float ndivsize);
  void DrawPlane(Plane *pPlane, int ndivs, float ndivsize);
  void DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys,
                         const std::vector<MapPoint *> &vMPs, cv::Mat &im);

  Plane *DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint *> &vMPs,
                     const int iterations = 50);

  // frame rate
  float mFPS, mT;
  float fx, fy, cx, cy;

  // Last processed image and computed pose by the SLAM
  std::mutex mMutexPoseImage;
  cv::Mat mTcw;
  cv::Mat mImage;
  int mStatus;
  std::vector<cv::KeyPoint> mvKeys;
  std::vector<MapPoint *> mvMPs;
};

}  // namespace ORB_SLAM2

#endif  // VIEWERAR_H
