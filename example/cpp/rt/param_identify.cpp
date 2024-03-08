/**
 * @file joint_s_line.cpp
 * @brief 实时模式 - 轴空间S规划
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
using namespace std;

void save_data(void){
    ofstream out_txt_file;
    out_txt_file.open("number.txt");
    int i;
    int arr[5] = {1, 2, 3, 4, 5};
    for (i = 0; i < 5; i++){
        out_txt_file << arr[i];
    }
    out_txt_file.close();
}
int main(){
    ofstream out_txt_file;
    ofstream torque_file;

    out_txt_file.open("data.txt");
    torque_file.open("torque.txt");

    try{
        std::string ip = "192.168.0.160";
        std::error_code ec;
        rokae::xMateErProRobot robot(ip, "192.168.0.180"); // 本机地址192.168.0.100

        robot.setOperateMode(rokae::OperateMode::automatic,ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);

        auto rtCon = robot.getRtMotionController().lock();

        // 设置要接收数据
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});
        std::array<double,7> jntPos{}, delta{};
        JointPosition cmd(7);

        static bool init = true;
        double time = 0;

        std::vector<std::array<double, 7>> jntTargets = {
            {0, -0.4, 0, 0, 0, 0, 0},
            {0, 0.4, 0, 0, 0, 0, 0},
            {0, -0.6, 0, 0, 0, 0, 0},
            {0, 0.6, 0, 0, 0, 0, 0},
            {0, -0.4, 0, 0, 0, 0, 0},
            {0, 0.6, 0, 0, 0, 0, 0},
            {0, -0.4, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0}
        };    
        auto it = jntTargets.begin();

        //开始运动前先设置为轴空间位置控制
        rtCon->startMove(RtControllerMode::jointPosition);

        std::function<JointPosition(void)> callback = [&, rtCon](){
            time += 0.001; // 按1ms为周期规划
            if(init) {
                // 读取当前轴角度
                jntPos = robot.jointPos(ec);
                init = false;
            }
            JointMotionGenerator joint_s(0.9, *it);
            joint_s.calculateSynchronizedValues(jntPos);
            print(std::cout, "joint angle: ", robot.jointPos(ec));
            print(std::cout, "joint Torque: ", robot.jointTorque(ec));
            out_txt_file << "Position: " << robot.jointPos(ec) << endl;
            torque_file << "Torque: " << robot.jointTorque(ec) << endl;


            // 获取每个周期计算的角度偏移
            if(!joint_s.calculateDesiredValues(time, delta)){
                // for(unsigned i = 0; i < cmd.joints.size(); ++i){
                for(unsigned i = 0; i < cmd.joints.size(); ++i){
                    cmd.joints[i] = jntPos[i] + delta[i];}
            }else{
                // 已到达一个目标点，开始运动到下一个目标点
                if (++it == jntTargets.end()){
                    cmd.setFinished();}
                time = 0;
                // 最后的角度值作为下一个规划的起始点
                std::copy(cmd.joints.begin(), cmd.joints.end(), jntPos.begin());
            }
            return cmd;
        };

        rtCon->setControlLoop(callback);
        rtCon->startLoop(true);
        print(std::cout, "控制结束");

    }catch (const std::exception &e) {
        print(std::cerr, e.what());}

    out_txt_file.close();
    torque_file.close();

    return 0;
}
