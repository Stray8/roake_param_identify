/**
 * @file move_commands.cpp
 * @brief 实时模式 - S规划MoveJ & MoveL & MoveC
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <cmath>
#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "../print_helper.hpp"
#include <fstream>


using namespace rokae;

std::ostream &os = std::cout;

int main() {
  using namespace std;
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    xMateErProRobot robot(ip, "192.168.0.180"); // ****   XMate 7-axis
    robot.setOperateMode(OperateMode::automatic,ec);

    // 若程序运行时控制器已经是实时模式，需要先切换到非实时模式后再更改网络延迟阈值，否则不生效
    robot.setRtNetworkTolerance(20, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock(); 
        //读取txt中的轨迹信息
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


    // 示例程序使用机型: xMateER7 Pro
    // 1. 从当前位置MoveJ运动到拖拽位置
    // std::array<double, 7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
    std::array<double, 7> q_drag_xm7p = {0, M_PI/4, 0, M_PI/4, 0, M_PI/4, 0};
    std::array<double, 7> q_next_xm7p = {0, 0, 0, 0, 0, 0, 0};
    // std::vector<std::array<double, 7>> jntTargets = {
    //   {0, 0, 0, 0, 0, 0, 0},
    //   {0, -0.4, 0.2, 0.2, 0.1, 0.2, 0},
    //   {0, 0.4, -0.2, 0, 0, 0, 0},
    //   {0, 0.6, 0, 0, 0, 0, 0},
    //   {0, -0.2, 0, 0, 0, 0, 0},
    //   {0, 0.4, 0, 0, 0, 0, 0},
    //   {0, 0.6, 0, 0, 0, 0, 0},
    //   {0, -0.4, 0, 0, 0, 0, 0}
    // };    
    // rtCon->MoveJ(0.4, robot.jointPos(ec), q_drag_xm7p);
    // print(os, "Now position:", robot.jointPos(ec));
    print(os, "real position:", q_next_xm7p);
    for (int i = 0; i < jntTargets.size(); i++){
      rtCon->MoveJ(0.4, robot.jointPos(ec), jntTargets[i]); 
      print(os, "Now position:", robot.jointPos(ec));

    }
    

    rtCon->MoveJ(0.4, robot.jointPos(ec), q_drag_xm7p); 
    rtCon->MoveJ(0.4, robot.jointPos(ec), q_next_xm7p); 



    // 4. 关闭实时模式
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);
  
    // // 笛卡尔空间，自由拖动
    // robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
    // // print("按回车继续");
    // std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
    // print(os, "打开拖动", ec, "按回车继续");
    // while(getchar() != '\n');
    // robot.disableDrag(ec);
    // std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式

    print(os, "done");

  } catch (const std::exception &e) {
    std::cerr << e.what();
  }
  return 0;
}
