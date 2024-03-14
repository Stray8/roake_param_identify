﻿/**
 * @file joint_position_control.cpp
 * @brief 实时模式 - 轴角度控制
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
#include <fstream>


using namespace rokae;

int main() {
  using namespace std;
  ofstream out_txt_file;
  ofstream torque_file;

  out_txt_file.open("position_po.txt");
  torque_file.open("torque_po.txt");
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.180"); // 本机地址192.168.0.100

    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();

    // 设置要接收数据。其中jointPos_m是本示例程序会用到的
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m, 
                                                                RtSupportedFields::jointVel_m, 
                                                                RtSupportedFields::tau_m});

    static bool init = true;
    double time = 0;

    std::array<double, 7> jntPos{};

    std::array<double, 7> tau{};
    robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
    std::array<double,7> q_drag_xm7p = {0, 0, 0, 0, 0, 0, 0};

    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.5, jntPos, q_drag_xm7p);
  
    ifstream file("/home/robot/robot/traj/dq.txt");

    vector<array<double, 7>> jntTargets;
    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        array<double, 7> arr;
        string value;
        for (int i = 0; i < 7; i++) {
            if (!getline(iss, value, ',')) {
                cerr << "Error" << line << endl;
                return 1;
            }
            arr[i] = stod(value);
        }
        jntTargets.push_back(arr);
    }
    file.close();


    // 开始轴空间位置控制
    rtCon->startMove(RtControllerMode::jointPosition);

    std::function<JointPosition()> callback = [&, rtCon](){
      if(init) {
        robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
        init = false;
      }
      robot.getStateData(RtSupportedFields::tau_m, tau);

      time += 0.001;
      double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / 2 * time));
      JointPosition cmd = {{jntPos[0] + delta_angle, jntPos[1] + delta_angle,
                            jntPos[2] + delta_angle, jntPos[3] + delta_angle,
                            jntPos[4] + delta_angle, jntPos[5] + delta_angle,
                            jntPos[6] + delta_angle}};

      
      out_txt_file << "Position: " << jntPos << endl;
      torque_file << "Torque: " << tau << endl;      
      print(cout, time);

      if(time > 10) {
        cmd.setFinished(); // 60秒后结束
      }
      return cmd;
    };

    rtCon->setControlLoop(callback);
    // 阻塞loop
    rtCon->startLoop(true);
    print(std::cout, "控制结束");
    out_txt_file.close();
    torque_file.close();

  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}
