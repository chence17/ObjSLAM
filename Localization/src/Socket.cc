/*
 * @Author: Antonio Chan
 * @Date: 2021-04-14 17:06:04
 * @LastEditTime: 2021-04-14 17:51:02
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/src/Socket.cc
 */

#include "Socket.h"

namespace ORB_SLAM2 {
    Socket::Socket(unsigned int initialFrameID, unsigned int maximumFrameID, unsigned int portNumber):
            mDataToServer(""),
            mDataFromServer(""),
            sock(this->ios),
            ep(boost::asio::ip::address::from_string("127.0.0.1"), portNumber),
            mReceive(false),
            mFinish(false),
            mFrameID(initialFrameID),
            mMaximumFrameID(maximumFrameID)
    {
        this->sock.connect(this->ep);
    }

    Socket::~Socket() {
        this->sock.send(boost::asio::buffer("quit_client"));
    }

    void Socket::Run() {
        while(1){
            if(!this->CheckFinish() && !this->CheckReceive()){
                std::memset(this->mDataToServer, 0, sizeof(this->mDataToServer));
                std::memset(this->mDataFromServer, 0, sizeof(this->mDataFromServer));
                std::string data = std::to_string(this->mFrameID);
                strcpy(this->mDataToServer, data.c_str());
                this->sock.send(boost::asio::buffer(this->mDataToServer));
                this->sock.receive(boost::asio::buffer(this->mDataFromServer));
                this->SetReceiveTrue();
            }else if(this->CheckFinish()){
                break;
            }
        }
    }

    std::vector<KeyObject> Socket::GetKeyObjects() {
        while(!this->CheckReceive()){}
        std::vector<KeyObject> keyObjects;
        json rawData = json::parse(this->mDataFromServer);
        for (auto& element : rawData) {
            if(element["type"].get<std::string>()=="Car"){
                auto length = element["length"].get<float>();
                auto width = element["width"].get<float>();
                auto height = element["height"].get<float>();
                auto theta = element["theta"].get<float>();
                bool valid = true;
                CenterPoints center;
                if(!element["center"].empty()){
                    center = element["center"].get<CenterPoints>();
                } else {
                    center = {0.0, 0.0, 0.0};
                    valid = false;
                }
                Box2DPoints box2D;
                if(!element["box3d_pts_2d"].empty()){
                    box2D = element["box3d_pts_2d"].get<Box2DPoints>();
                } else {
                    for(auto& pt : box2D) {
                        pt = {0.0, 0.0};
                    }
                    valid = false;
                }
                keyObjects.emplace_back(center, length, width, height,
                                        theta, box2D, valid);
            }
        }
        std::cout << "Data of frame " << this->mFrameID << " is received." << std::endl;
        if(mFrameID<mMaximumFrameID){
            this->mFrameID+=1;
            this->SetReceiveFalse();
        }
        return keyObjects;
    }

    bool Socket::CheckReceive() {
        std::unique_lock<std::mutex> lock(this->mMutexReceive);
        return this->mReceive;
    }

    void Socket::SetReceiveTrue() {
        std::unique_lock<std::mutex> lock(this->mMutexReceive);
        this->mReceive=true;
    }

    void Socket::SetReceiveFalse() {
        std::unique_lock<std::mutex> lock(this->mMutexReceive);
        this->mReceive=false;
    }

    bool Socket::CheckFinish() {
        std::unique_lock<std::mutex> lock(this->mMutexFinish);
        return this->mFinish;
    }

    void Socket::SetFinish() {
        std::unique_lock<std::mutex> lock(this->mMutexFinish);
        this->mFinish=true;
    }

}  // namespace ORB_SLAM2
