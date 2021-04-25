/*
 * @Author: Antonio Chan
 * @Date: 2021-04-20 10:33:18
 * @LastEditTime: 2021-04-20 10:50:34
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/KeyObject.h
 */

#ifndef INC_KEYOBJECT_H_
#define INC_KEYOBJECT_H_

#include <vector>
#include <array>
#include <opencv2/core/core.hpp>
#include <cmath>

namespace ORB_SLAM2 {
    typedef std::array<float, 3> CenterPoints;
    typedef std::array<std::array<float, 2>, 8> Box2DPoints;
    class KeyObject {
    public:
        KeyObject(const CenterPoints& center, float length, float width, float height, float theta, const Box2DPoints& box2D, bool valid);
        KeyObject(const KeyObject& ko);

        CenterPoints mCenter{};
        float mLength;
        float mWidth;
        float mHeight;
        float mTheta;
        Box2DPoints mBox2D{};
        cv::Mat mTcw;

        bool mValid;

        bool SetTcw(const cv::Mat& Tcw);
        cv::Mat GetRotation();
        cv::Mat GetTranslation();
};

    double ComputeObjectPoseDistance(const cv::Mat& SrcObj, const cv::Mat& SrcFrameTcw, const cv::Mat& DstObj, const cv::Mat& DstFrameTcw);

}  // namespace ORB_SLAM2

#endif  // INC_KEYOBJECT_H_
