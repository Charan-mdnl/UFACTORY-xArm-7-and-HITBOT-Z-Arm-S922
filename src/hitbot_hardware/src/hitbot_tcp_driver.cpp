// Copyright (c) 2026, Chethan
// BSD-3-Clause License
//
// Implementation of the standalone HITBOT TCP driver.
// This driver uses the HITBOT Z-Arm S922 BINARY protocol on port 8081.
// The protocol envelope is: /f/bIII<payload>
// Responses contain binary float64 (little-endian) joint data.

#include "hitbot_hardware/hitbot_tcp_driver.hpp"

#include <arpa/inet.h>
#include <csignal>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace hitbot_hardware {

// ────────────────────────────────────────────────────────────────────────────
// Construction / destruction
// ────────────────────────────────────────────────────────────────────────────

HitbotTcpDriver::HitbotTcpDriver() = default;

HitbotTcpDriver::~HitbotTcpDriver() { disconnect(); }

// ────────────────────────────────────────────────────────────────────────────
// Connection management
// ────────────────────────────────────────────────────────────────────────────

int HitbotTcpDriver::connect(const std::string& ip, int port) {
  // Suppress SIGPIPE so broken TCP connections don't kill the process.
  std::signal(SIGPIPE, SIG_IGN);

  sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0) {
    std::cerr << "[HitbotTcpDriver] Socket creation error\n";
    return -1;
  }

  // Set a 2-second receive timeout so we don't block forever.
  struct timeval tv;
  tv.tv_sec  = 2;
  tv.tv_usec = 0;
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  struct sockaddr_in serv_addr{};
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port   = htons(port);

  if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
    std::cerr << "[HitbotTcpDriver] Invalid address: " << ip << "\n";
    ::close(sock_);
    sock_ = -1;
    return -1;
  }

  if (::connect(sock_, reinterpret_cast<struct sockaddr*>(&serv_addr),
                sizeof(serv_addr)) < 0) {
    std::cerr << "[HitbotTcpDriver] Connection to " << ip << ":" << port
              << " failed\n";
    ::close(sock_);
    sock_ = -1;
    return -1;
  }

  std::cout << "[HitbotTcpDriver] Connected to " << ip << ":" << port << "\n";
  msgCounter_ = 1;
  return 0;
}

void HitbotTcpDriver::disconnect() {
  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
}

bool HitbotTcpDriver::isConnected() const { return sock_ >= 0; }

// ────────────────────────────────────────────────────────────────────────────
// Protocol helpers
// ────────────────────────────────────────────────────────────────────────────

void HitbotTcpDriver::sendCommand(int instructionID,
                                   const std::string& command) {
  if (sock_ < 0) return;

  std::string packet =
      std::string(kStartHeader) + kSeparator +
      std::to_string(msgCounter_) + kSeparator +
      std::to_string(instructionID) + kSeparator +
      std::to_string(command.length()) + kSeparator +
      command + kSeparator + kEndHeader;

  ssize_t sent = ::send(sock_, packet.c_str(), packet.size(), MSG_NOSIGNAL);
  if (sent < 0) {
    std::cerr << "[HitbotTcpDriver] Send failed, connection lost\n";
    disconnect();
    return;
  }
  msgCounter_++;

  std::memset(buffer_, 0, sizeof(buffer_));
  bytes_received_ = 0;
  
  // Read until we get the end header "/b/f" or timeout
  while (bytes_received_ < sizeof(buffer_) - 1) {
    int bytes_read = ::read(sock_, buffer_ + bytes_received_, sizeof(buffer_) - 1 - bytes_received_);
    if (bytes_read <= 0) {
      std::cerr << "[HitbotTcpDriver] Read failed, connection lost\n";
      disconnect();
      return;
    }
    bytes_received_ += bytes_read;
    buffer_[bytes_received_] = '\0';
    
    // Check if we have the end header
    std::string current_buf(buffer_, bytes_received_);
    if (current_buf.find(kEndHeader) != std::string::npos) {
      break;
    }
  }
}

void HitbotTcpDriver::sendSpecialCommand(int instructionID,
                                          const std::string& command) {
  if (sock_ < 0) return;

  std::string packet =
      std::string(kStartHeader) + kSeparator +
      std::to_string(msgCounter_) + kSeparator +
      std::to_string(instructionID) + kSeparator +
      std::to_string(command.length()) + kSeparator +
      command + kSeparator + kEndHeader;

  ssize_t sent = ::send(sock_, packet.c_str(), packet.size(), MSG_NOSIGNAL);
  if (sent < 0) { disconnect(); return; }
  msgCounter_++;

  std::memset(buffer_, 0, sizeof(buffer_));
  int bytes_read = ::read(sock_, buffer_, sizeof(buffer_));
  bytes_received_ = (bytes_read > 0) ? bytes_read : 0;
  if (bytes_read <= 0) { disconnect(); return; }
  // Some commands need a second read
  bytes_read = ::read(sock_, buffer_, sizeof(buffer_));
  if (bytes_read > 0) bytes_received_ = bytes_read;
}

// ── Text-based response parsers (for controllers that use text protocol) ──

std::vector<std::string> HitbotTcpDriver::getContent() {
  std::string packet(buffer_, bytes_received_);
  
  // Example packet: /f/bIII1III501III62III104.5,-44.8,30.4,-135.3,0.5,51.3III/b/f
  size_t content_start = 0;
  for (int i = 0; i < 4; ++i) {
    content_start = packet.find(kSeparator, content_start);
    if (content_start == std::string::npos) return {};
    content_start += 3; // length of "III"
  }
  
  size_t content_end = packet.find(kSeparator, content_start);
  if (content_end == std::string::npos) return {};
  
  std::string content = packet.substr(content_start, content_end - content_start);
  
  std::vector<std::string> data;
  size_t pos = 0;
  while ((pos = content.find(",")) != std::string::npos) {
    data.push_back(content.substr(0, pos));
    content.erase(0, pos + 1);
  }
  if (!content.empty()) {
    data.push_back(content);
  }
  
  return data;
}

bool HitbotTcpDriver::getContentBool() {
  auto v = getContent();
  return (v.size() == 1 && v[0] == "1");
}

std::vector<float> HitbotTcpDriver::getContentFloat() {
  auto v = getContent();
  std::vector<float> result;
  for (const auto& s : v) {
    try {
      result.push_back(std::stof(s));
    } catch(...) {
      result.push_back(0.0f);
    }
  }
  return result;
}

// ────────────────────────────────────────────────────────────────────────────
// Joint-state queries
// ────────────────────────────────────────────────────────────────────────────

std::vector<float> HitbotTcpDriver::getJointPosDegree() {
  sendCommand(GetActualJointPosDegree_ID, "GetActualJointPosDegree()");
  return getContentFloat();
}

std::vector<float> HitbotTcpDriver::getJointPosRadian() {
  auto deg = getJointPosDegree();
  std::vector<float> rad(deg.size());
  for (size_t i = 0; i < deg.size(); ++i) {
    rad[i] = deg[i] * static_cast<float>(M_PI / 180.0);
  }
  return rad;
}

std::vector<float> HitbotTcpDriver::getJointSpeedsDegree() {
  sendCommand(GetActualJointSpeedsDegree_ID, "GetActualJointSpeedsDegree()");
  return getContentFloat();
}

std::vector<float> HitbotTcpDriver::getActualTCPPose() {
  sendCommand(GetActualTCPPose_ID, "GetActualTCPPose()");
  return getContentFloat();
}

// ────────────────────────────────────────────────────────────────────────────
// Motion commands
// ────────────────────────────────────────────────────────────────────────────

bool HitbotTcpDriver::servoJ(const std::vector<float>& jointAngle,
                              float acc, float vel, float t,
                              float lookaheadTime, float gain) {
  std::string cmd = "ServoJ(";
  for (auto a : jointAngle) cmd += std::to_string(a) + ",";
  cmd += std::to_string(acc) + "," + std::to_string(vel) + "," +
         std::to_string(t) + "," + std::to_string(lookaheadTime) + "," +
         std::to_string(gain) + ")";
  sendCommand(ServoJ_ID, cmd);
  return true;  // Binary protocol may not return standard bool
}

bool HitbotTcpDriver::moveJoint(const std::vector<float>& jointAngles,
                                 float speed, float acc) {
  std::string cmd = "MoveJ(";
  for (auto a : jointAngles) cmd += std::to_string(a) + ",";
  for (int i = 0; i < 6; ++i) cmd += "0.0,";
  cmd += "0,0," + std::to_string(speed) + "," + std::to_string(acc) + "," +
         std::to_string(speed) + ",";
  cmd += "0.0,0.0,0.0,0.0,";
  cmd += "0.0,0,0.0,0.0,0.0,0.0,0.0,0.0)";
  sendCommand(MoveJ_ID, cmd);
  return true;
}

// ────────────────────────────────────────────────────────────────────────────
// Configuration commands
// ────────────────────────────────────────────────────────────────────────────

bool HitbotTcpDriver::resetErrors() {
  sendCommand(RESETALLERROR_ID, "RESETALLERROR");
  return true;
}

bool HitbotTcpDriver::switchMode(int mode) {
  sendCommand(Mode_ID, "Mode(" + std::to_string(mode) + ")");
  return true;
}

bool HitbotTcpDriver::setSpeed(uint8_t speed) {
  sendCommand(SetSpeed_ID, "SetSpeed(" + std::to_string(speed) + ")");
  return true;
}

bool HitbotTcpDriver::switchDragTeach(uint8_t status) {
  sendCommand(DragTeachSwitch_ID,
              "DragTeachSwitch(" + std::to_string(status) + ")");
  return true;
}

}  // namespace hitbot_hardware
