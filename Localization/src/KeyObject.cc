#include "KeyObject.h"

namespace ORB_SLAM2 {
    KeyObject::KeyObject(const CenterPoints& center, float length, float width, float height, float theta, const Box2DPoints& box2D) :
            mCenter(center), mLength(length), mWidth(width), mHeight(height), mTheta(theta), mBox2D(box2D) {
        this->mTcw.create(4, 4, CV_32F);
        this->mTcw.at<float>(0, 0) = std::cos(this->mTheta);
        this->mTcw.at<float>(0, 1) = 0.0;
        this->mTcw.at<float>(0, 2) = std::sin(this->mTheta);
        this->mTcw.at<float>(0, 3) = -this->mCenter[0];
        this->mTcw.at<float>(1, 0) = 0.0;
        this->mTcw.at<float>(1, 1) = 1.0;
        this->mTcw.at<float>(1, 2) = 0.0;
        this->mTcw.at<float>(1, 3) = -this->mCenter[1];
        this->mTcw.at<float>(2, 0) = -std::sin(this->mTheta);
        this->mTcw.at<float>(2, 1) = 0.0;
        this->mTcw.at<float>(2, 2) = std::cos(this->mTheta);
        this->mTcw.at<float>(2, 3) = -this->mCenter[2];
        this->mTcw.at<float>(3, 0) = 0.0;
        this->mTcw.at<float>(3, 1) = 0.0;
        this->mTcw.at<float>(3, 2) = 0.0;
        this->mTcw.at<float>(3, 3) = 1.0;
    }

    KeyObject::KeyObject(const KeyObject &ko) {
        this->mCenter = ko.mCenter;
        this->mLength = ko.mLength;
        this->mWidth = ko.mWidth;
        this->mHeight = ko.mHeight;
        this->mTheta = ko.mTheta;
        this->mBox2D = ko.mBox2D;
        this->mTcw = ko.mTcw;
    }
} // namespace ORB_SLAM2