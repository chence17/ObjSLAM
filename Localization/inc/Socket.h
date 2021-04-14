/*
 * @Author: Antonio Chan
 * @Date: 2021-04-14 17:05:55
 * @LastEditTime: 2021-04-14 17:50:07
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/Socket.h
 */

// https://blog.csdn.net/shyjhyp11/article/details/109891396
// https://blog.csdn.net/qq_27664167/article/details/103312462

#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

namespace ORB_SLAM2 {
class Socket {
 public:
  Socket();
  ~Socket();
  json Receive();

 private:
  //定义一个缓冲区
  // char data[128];
  char msgToServer[256];
  char msgFromServer[16384];

  // io_service对象
  boost::asio::io_service ios;
  //创建socket对象
  boost::asio::ip::tcp::socket sock;
  //创建连接端
  boost::asio::ip::tcp::endpoint ep;

  // Frame ID
  int i;
};

}  // namespace ORB_SLAM2
