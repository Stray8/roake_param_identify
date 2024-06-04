/**
 * @file joint_impedance_control.cpp
 * @brief 实时模式 - 轴空间阻抗控制
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "../print_helper.hpp"

using namespace rokae;

int main() {
  using namespace std;
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.180"); // 本机地址192.168.0.100

    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();

    // 设置要接收数据。其中jointPos_m是本示例程序会用到的
    robot.startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::jointPos_m});

    static bool init = true;
    double time = 0;

    std::array<double, 7> jntPos{};
    robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};

    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.4, jntPos, q_drag_xm7p);

    cout << "done" << endl;
  } catch (const std::exception &e) {
    std::cout << e.what();
  }
  return 0;
}
