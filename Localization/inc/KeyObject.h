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
        KeyObject(CenterPoints center, float length, float width, float height, float theta, Box2DPoints box2D);
        KeyObject(const KeyObject& ko);

        CenterPoints mCenter{};
        float mLength;
        float mWidth;
        float mHeight;
        float mTheta;
        Box2DPoints mBox2D{};
        cv::Mat mTcw;
};

}  // namespace ORB_SLAM2

#endif  // INC_KEYOBJECT_H_
