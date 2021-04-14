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
Socket::Socket()
    : sock(ios),
      ep(boost::asio::ip::address::from_string("127.0.0.1"), 6783),
      i(0) {
  this->sock.connect(ep);
}

Socket::~Socket() {
  this->sock.send(boost::asio::buffer("quit_client"));
  // sock.send(boost::asio::buffer("quit_server"));
}

json Socket::Receive() {
  std::memset(this->msgToServer, 0, sizeof(this->msgToServer));
  std::memset(this->msgFromServer, 0, sizeof(this->msgFromServer));

  std::string tmp =
      "Request inference of frame " + std::to_string(this->i) + ".";
  strcpy(this->msgToServer, tmp.c_str());

  std::cout << this->msgToServer << std::endl;

  // boost::asio::write(sock,boost::asio::buffer(msgToServer));
  this->sock.send(boost::asio::buffer(this->msgToServer));

  // writing server dealing with the message...

  // boost::asio::read(sock,boost::asio::buffer(msgFromServer));
  this->sock.receive(boost::asio::buffer(this->msgFromServer));
  std::cout << "Inference of frame ";
  std::cout << this->i << " received." << std::endl;
  this->i++;
  return json::parse(this->msgFromServer);
}
}  // namespace ORB_SLAM2
