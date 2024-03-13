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

int main(){
    ofstream position_file;
    ofstream torque_file;
    ofstream velociy_file;

    position_file.open("position.txt");
    torque_file.open("torque.txt");
    velociy_file.open("velocity.txt");


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

    try{
        string ip = "192.168.0.160";
        error_code ec;
        xMateErProRobot robot(ip, "192.168.0.180"); // 本机地址192.168.0.100

        robot.setOperateMode(rokae::OperateMode::automatic,ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);

        auto rtCon = robot.getRtMotionController().lock();

        // 设置要接收数据
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m, RtSupportedFields::jointVel_m, RtSupportedFields::tau_m});

        array<double,7> jntPos{}, delta{}, jntVel{}, tau{};
        JointPosition cmd(7);

        static bool init = true;
        double time = 0;

        auto it = jntTargets.begin(); //返回指向jntTargets的第一个元素的迭代器

        // 开始运动前先设置为轴空间位置控制
        rtCon->startMove(RtControllerMode::jointPosition);

        function<JointPosition(void)> callback = [&, rtCon](){
            if(init){
                robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
                robot.getStateData(RtSupportedFields::jointVel_m, jntVel);
                robot.getStateData(RtSupportedFields::tau_m, tau);

                init = false;
            }
            time += 0.001;

            JointMotionGenerator joint_s(1, *it);
            joint_s.calculateSynchronizedValues(jntPos);
            print(std::cout, "joint angle: ", robot.jointPos(ec));
            // print(std::cout, "joint Torque: ", robot.jointTorque(ec));
            position_file << "Position: " << jntPos << endl;
            torque_file << "Torque: " << jntVel << endl;
            velociy_file << "velocity: " << tau << endl;


            // print(cout, "start:", time);
            // print(cout, "it:", *it);
            // print(cout, joint_s.calculateDesiredValues(time, delta));

            // 获取每个周期计算的角度偏移
            if(!joint_s.calculateDesiredValues(time, delta)){
                for(unsigned i = 0; i < cmd.joints.size(); ++i)
                    cmd.joints[i] = jntPos[i] + delta[i];
            }else{
                // 已到达一个目标点，开始运动到下一个目标点
                if (++it == jntTargets.end())
                    cmd.setFinished();

                // print(cout, "over: ", time);
                time = 0;
                // 最后的角度值作为下一个规划的起始点
                copy(cmd.joints.begin(), cmd.joints.end(), jntPos.begin());
            }
            return cmd;
        };

        rtCon->setControlLoop(callback);
        rtCon->startLoop(true);
        print(cout, "控制结束");
        position_file.close();
        torque_file.close();
        velociy_file.close();

        // 关闭实时模式
        robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        robot.setOperateMode(rokae::OperateMode::manual, ec);
    }catch (const std::exception &e) {
        print(std::cerr, e.what());
    }
    return 0;
}

