/**
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
#include "Eigen/Geometry"
#include "../print_helper.hpp"
#include "rokae/utility.h"

using namespace rokae;
using namespace std;

int main() {
  // 初始化
  ofstream position_file;
  ofstream velocity_file;
  ofstream accelation_file;
  // ofstream full_file;
  ofstream gravity_file;
  // ofstream inertia_file;
  // ofstream coriolis_file;
  ofstream torque_file;
  ofstream jp_file;
  ofstream jv_file;
  ofstream tau_file;
  position_file.open("position_ly.txt");
  velocity_file.open("velocity_ly.txt");
  accelation_file.open("accelation_ly.txt");
  // full_file.open("full_ly.txt");
  // inertia_file.open("inertia_ly.txt");
  // coriolis_file.open("coriolis_ly.txt");
  gravity_file.open("gravity_ly.txt");
  torque_file.open("torque_ly.txt");
  jp_file.open("jp_ly.txt");
  jv_file.open("jv_ly.txt");
  tau_file.open("tau_ly.txt");

  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;

    rokae::xMateErProRobot robot(ip, "192.168.0.180"); // 本机地址192.168.0.100
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();
    auto model = robot.model();

    robot.stopReceiveRobotState();
    // 设置要接收数据。其中jointPos_m是本示例程序会用到的
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m, 
                                                                RtSupportedFields::jointPos_c, 
                                                                RtSupportedFields::jointVel_m, 
                                                                RtSupportedFields::tau_m,
                                                                RtSupportedFields::jointAcc_c});

    static bool init = true;


    double time = 0;

    std::array<double, 7> jntPos{}, jntVel{};

    robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
    // std::array<double,7> q_drag_xm7p = {-M_PI/15, -M_PI/15, -M_PI/15, -M_PI/15, -M_PI/15, -M_PI/15,- M_PI/15};
    // std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
    std::array<double,7> q_drag_xm7p = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> q{}, ddq_c{}, dq_m{}, tau{}, jp{}, jv{}, tor{}, full_array{}, inertia_array{}, coriolis_array{}, gravity_array{}, ja{}, jv_before{};

    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.2, jntPos, q_drag_xm7p);

    // 开始轴空间位置控制
    rtCon->startMove(RtControllerMode::jointPosition);

    std::function<JointPosition()> callback = [&, rtCon](){
      if(init) {
        robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
        init = false;
      }

      time += 0.001;
      robot.updateRobotState(chrono::milliseconds(1));
      robot.getStateData(RtSupportedFields::jointPos_m, q);
      robot.getStateData(RtSupportedFields::jointVel_m, dq_m);
      robot.getStateData(RtSupportedFields::jointAcc_c, ddq_c);
      robot.getStateData(RtSupportedFields::tau_m, tau);

      jp = robot.jointPos(ec);
      jv = robot.jointVel(ec);
      tor = robot.jointTorque(ec);

      // std::array<double, 7> full_array = model.getTorque(q, dq_m, ddq_c, TorqueType::full);
      // std::array<double, 7> inertia_array = model.getTorque(q, dq_m, ddq_c, TorqueType::inertia);
      // std::array<double, 7> coriolis_array = model.getTorque(q, dq_m, ddq_c, TorqueType::coriolis);
      // std::array<double, 7> gravity_array = model.getTorque(q, dq_m, ddq_c, TorqueType::gravity);

      double delta_angle0 = M_PI / 20.0 * (1 - std::cos(M_PI * time));
      double delta_angle1 = M_PI / 20.0 * (1 - std::cos(M_PI / 2 * time));
      double delta_angle2 = M_PI / 20.0 * (1 - std::cos(M_PI * time));
      double delta_angle3 = M_PI / 20.0 * (1 - std::cos(M_PI / 2 * time));
      double delta_angle4 = M_PI / 20.0 * (1 - std::cos(M_PI / 4 * time));
      double delta_angle5 = M_PI / 20.0 * (1 - std::cos(M_PI / 2 * 3 * time));
      double delta_angle6 = M_PI / 20.0 * (1 - std::cos(M_PI / 4 * 3 * time));

      // double delta_angle1 = M_PI / 25.0 * (1 - std::cos(M_PI * time));
      // double delta_angle2 = M_PI / 30.0 * (1 - std::sin(M_PI / 3 * time));
      // double delta_angle3 = M_PI / 20.0 * std::sin(M_PI / 2 * time);
      // double delta_angle4 = M_PI / 20.0 * (1 - std::sin(M_PI * time + M_PI / 2));
      // double delta_angle5 = -M_PI / 30.0 * std::sin(M_PI / 6 * time);
      // double delta_angle6 = M_PI / 35.0 * (1 + std::sin(M_PI / 2 * 3 * time - M_PI /2));

      if(time > 8){
        delta_angle0 = 0;
        delta_angle1 = 0;
        delta_angle2 = 0;
        delta_angle3 = 0;
        delta_angle4 = 0;
        delta_angle5 = 0;
        delta_angle6 = 0;
      }

      JointPosition cmd = {{jntPos[0] + delta_angle0, jntPos[1] + delta_angle1,
                            jntPos[2] - delta_angle2, jntPos[3] - delta_angle3,
                            jntPos[4] + delta_angle4, jntPos[5] + delta_angle5,
                            jntPos[6] + delta_angle6}};
      // JointPosition cmd = {{jntPos[0] + delta_angle0, jntPos[0] + delta_angle0,
      //                       jntPos[0] + delta_angle0, jntPos[0] + delta_angle0,
      //                       jntPos[0] + delta_angle0, jntPos[0] + delta_angle0,
      //                       jntPos[0] + delta_angle0}};

      // 保存数据
      position_file << q << endl;
      velocity_file << dq_m << endl;
      accelation_file << ddq_c << endl;
      // full_file << full_array << endl;
      // inertia_file << inertia_array << endl;
      // coriolis_file << coriolis_array << endl;
      gravity_file << gravity_array << endl;
      jp_file << jp << endl;
      jv_file << jv << endl;
      torque_file << tor << endl;
      tau_file << tau << endl;

      // cout << "tau: " << tau << endl;
      // cout << "torque:" << tor << endl;
      // cout << time << endl;
    
      if(time > 10)
        cmd.setFinished(); // 60秒后结束
      return cmd;
    };

    rtCon->setControlLoop(callback);
    // 阻塞loop
    rtCon->startLoop(true);
    print(std::cout, "控制结束");
    position_file.close();
    velocity_file.close();
    accelation_file.close();
    // full_file.close();
    // inertia_file.close();
    // coriolis_file.close();
    gravity_file.close();
    jp_file.close();
    jv_file.close();
    torque_file.close();
    tau_file.close();


  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}
