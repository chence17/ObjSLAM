/*
 * @Author: Antonio Chan
 * @Date: 2021-04-14 17:05:55
 * @LastEditTime: 2021-04-20 14:58:32
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/Socket.h
 */

// https://blog.csdn.net/shyjhyp11/article/details/109891396
// https://blog.csdn.net/qq_27664167/article/details/103312462

#ifndef INC_SOCKET_H_
#define INC_SOCKET_H_

#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "KeyObject.h"

using json = nlohmann::json;

namespace ORB_SLAM2 {
    class KeyObject;

    class Socket {
    public:
        Socket(unsigned int initialFrameID, unsigned int maximumFrameID, unsigned int portNumber=6783);
        ~Socket();
        void Run();
        std::vector<KeyObject> GetKeyObjects();
        void SetFinish();

    private:
        //定义一个缓冲区
        char mDataToServer[256]{};
        char mDataFromServer[16384]{};

        // io_service对象
        boost::asio::io_service ios;
        //创建socket对象
        boost::asio::ip::tcp::socket sock;
        //创建连接端
        boost::asio::ip::tcp::endpoint ep;

        // Receive Status
        bool mReceive;
        // Stop
        bool mFinish;
        // Frame ID
        unsigned int mFrameID;
        // Maximum Frame ID
        unsigned int mMaximumFrameID;

        // Tread flag
        std::mutex mMutexReceive;
        std::mutex mMutexFinish;

        void SetReceiveTrue();
        void SetReceiveFalse();
        bool CheckReceive();
        bool CheckFinish();
    };

}  // namespace ORB_SLAM2

#endif  // INC_SOCKET_H_
