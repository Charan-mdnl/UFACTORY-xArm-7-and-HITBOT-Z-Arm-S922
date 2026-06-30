// Copyright (c) 2026, Chethan
// BSD-3-Clause License
//
// Standalone TCP driver for the HITBOT Z-Arm S922 controller.
// Uses the same wire protocol as kelo_hitbot_driver (LGPL-2.1 / BSD):
//   /f/bIII<seq>III<instructionType>III<len>III<command>III/b/f
//
// This file has ZERO ROS dependencies – it is pure POSIX sockets + STL.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace hitbot_hardware {

class HitbotTcpDriver {
public:
  HitbotTcpDriver();
  ~HitbotTcpDriver();

  /// Open a TCP connection to the HITBOT controller.
  /// @return 0 on success, -1 on failure.
  int connect(const std::string& ip, int port);

  /// Close the TCP connection.
  void disconnect();

  /// @return true if the socket is connected.
  bool isConnected() const;

  // ── Joint state queries ──────────────────────────────────────────────
  /// Get the current joint angles in degrees (6 values).
  std::vector<float> getJointPosDegree();

  /// Get the current joint angles in radians (6 values).
  std::vector<float> getJointPosRadian();

  /// Get the current joint speeds in degrees/s (6 values).
  std::vector<float> getJointSpeedsDegree();

  /// Get the current TCP pose [x,y,z,rx,ry,rz] (mm / deg).
  std::vector<float> getActualTCPPose();

  // ── Motion commands ──────────────────────────────────────────────────
  /// Servo-move to a joint position (real-time streaming).
  /// @param jointAngle 6 joint angles in degrees.
  /// @param acc acceleration percentage 0–100.
  /// @param vel velocity percentage 0–100.
  /// @param t instruction cycle in seconds.
  /// @param lookaheadTime filter time (currently unused by controller).
  /// @param gain proportional gain (currently unused by controller).
  bool servoJ(const std::vector<float>& jointAngle,
              float acc, float vel, float t,
              float lookaheadTime, float gain);

  /// Point-to-point joint move.
  bool moveJoint(const std::vector<float>& jointAngles,
                 float speed, float acc);

  /// Reset all error flags on the controller.
  bool resetErrors();

  /// Switch operation mode: 0 = automatic, 1 = manual.
  bool switchMode(int mode);

  /// Set the global speed override (0–100 %).
  bool setSpeed(uint8_t speed);

  /// Enable / disable drag-teach mode.
  bool switchDragTeach(uint8_t status);

private:
  // ── Protocol helpers ─────────────────────────────────────────────────
  void sendCommand(int instructionID, const std::string& command);
  void sendSpecialCommand(int instructionID, const std::string& command);
  std::vector<std::string> getContent();
  bool getContentBool();
  std::vector<float> getContentFloat();
  std::vector<float> getContentFloatBinary(int offset, int count);

  int sock_{-1};
  size_t bytes_received_{0};
  char buffer_[2048]{};
  unsigned int msgCounter_{1};

  static constexpr const char* kStartHeader = "/f/b";
  static constexpr const char* kEndHeader   = "/b/f";
  static constexpr const char* kSeparator   = "III";

  // Command IDs (reverse-engineered from kelo_hitbot_driver).
  // These are the instruction-type integers sent in the packet header.
  enum CommandID {
    MoveJ_ID                     = 201,
    MoveL_ID                     = 203,
    MoveC_ID                     = 202,
    ServoJ_ID                    = 376,
    StartJOG_ID                  = 232,
    STOPJOG_ID                   = 233,
    STOPLINE_ID                  = 234,
    STOPTOOL_ID                  = 235,
    STOPWORKPIECE_ID             = 241,
    START_ID                     = 101,
    STOP_ID                      = 102,
    PAUSE_ID                     = 103,
    RESUME_ID                    = 104,
    RESETALLERROR_ID             = 107,
    Mode_ID                      = 303,
    SetSpeed_ID                  = 206,
    DragTeachSwitch_ID           = 333,
    SetDO_ID                     = 204,
    GetDI_ID                     = 212,
    SetAO_ID                     = 209,
    GetAI_ID                     = 214,
    SetToolDO_ID                 = 210,
    GetToolDI_ID                 = 213,
    SetToolAO_ID                 = 211,
    GetToolAI_ID                 = 215,
    WaitDI_ID                    = 218,
    WaitAI_ID                    = 220,
    WaitToolDI_ID                = 219,
    WaitToolAI_ID                = 221,
    WaitMs_ID                    = 207,
    GetActualJointPosDegree_ID   = 377,
    GetActualJointPosRadian_ID   = 377,
    GetActualJointSpeedsDegree_ID= 377,
    GetActualTCPPose_ID          = 377,
    GetForwardKin_ID             = 377,
    GetInverseKin_ID             = 377,
  };
};

}  // namespace hitbot_hardware
