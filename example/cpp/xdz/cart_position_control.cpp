/**
 * @file cartesian_impedance_control.cpp
 * @brief 实时模式 - 笛卡尔阻抗控制
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "Eigen/Geometry"
#include "../print_helper.hpp"
#include <fstream>

using namespace rokae;
using namespace std;
using namespace RtSupportedFields;

int main() {
    ofstream cart_posx_file;  
    ofstream cart_posy_file;    
    cart_posx_file.open("cart_posx.txt");
    cart_posy_file.open("cart_posy.txt");
    try {
        std::string ip = "192.168.0.160";
        std::error_code ec;
        rokae::xMateErProRobot robot(ip, "192.168.0.180"); // ****   xMate 7-axis
        robot.setRtNetworkTolerance(50, ec);
        robot.setOperateMode(rokae::OperateMode::automatic,ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);

        auto rtCon = robot.getRtMotionController().lock();
        robot.stopReceiveRobotState();
        robot.startReceiveRobotState(std::chrono::milliseconds(1),{jointPos_m, 
                                                                    jointVel_m, 
                                                                    jointAcc_c, 
                                                                    tcpPose_m,
                                                                    tcpPoseAbc_m, 
                                                                    tauExt_inBase});
        std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0 };
        rtCon->MoveJ(0.4, robot.jointPos(ec), q_drag_xm7p);
        cout << "move done!" << endl;
        // 设置力控坐标系为工具坐标系, 末端相对法兰的坐标系
        std::array<double, 16> toolToFlange = {0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1};
        rtCon->setFcCoor(toolToFlange, ForceControlFrameType::tool, ec);

        std::array<double, 16> init_position{};
        // Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), init_position);
        robot.getStateData(tcpPose_m, init_position);

        rtCon->startMove(RtControllerMode::cartesianPosition);
        array<double,6> cart_pos{};
        double time = 0;
        double t_ext = 0;
        // std::atomic<bool> stopManually {true};
        std::function<CartesianPosition(void)> callback = [&, rtCon]()->CartesianPosition{
            time += 0.001;
            robot.getStateData(tcpPoseAbc_m, cart_pos);
            /*
            走圆计算
            (x - 起始x位置)^2 + (y - 起始y位置)^2 = r^2
            x = rcos() + 起始x位置
            y = rsin() + 起始y位置
            起始x位置 + x增量 = x 反求x增量 直接给x位置会报错
            */
            constexpr double kRadius = 0.1;
            double delta_x = kRadius * std::cos(M_PI / 4 * time);
            double delta_y = kRadius * std::sin(M_PI / 4 * time);

            CartesianPosition output{};
            output.pos = init_position;
            output.pos[3] += delta_x - 0.1;
            output.pos[7] += delta_y;
            output.pos[11] += 1 / 5 * t_ext;
            if(time >= 5){
                t_ext = 5;
            }

            //保存坐标
            cart_posx_file << cart_pos[0] << endl;
            cart_posy_file << cart_pos[1] << endl;
            cout << "time: " << time << endl;
            cout << "position: " << cart_pos << endl;


            if(time > 20){
                std::cout << "运动结束" <<std::endl;
                output.setFinished();
                // stopManually.store(false); // loop为非阻塞，和主线程同步停止状态
            }
            return output;
        };
        rtCon->setControlLoop(callback, 0, true);
        rtCon->startLoop(true);   
        cart_posx_file.close();
        cart_posy_file.close();  
        // rtCon->setControlLoop(callback);
        // rtCon->startLoop(false);
        // while(stopManually.load());
        // rtCon->stopLoop();
        // std::this_thread::sleep_for(std::chrono::seconds(2));
    } catch (const std::exception &e) {
        std::cerr << e.what();
    }
    return 0;
}
