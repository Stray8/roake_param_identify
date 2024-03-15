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
using namespace Eigen;

#define PERIOD 0.001
#define CNT_PNTS 20000
#define RAD_FREQ 0.3141592653	// 2 * 0.1*PI 2*0.05*PI


int main(){
    ofstream position_file;
    ofstream torque_file;
    ofstream velociy_file;
    position_file.open("position.txt");
    torque_file.open("torque.txt");
    velociy_file.open("velocity.txt");


    string ip = "192.168.0.160";
    error_code ec;
    xMateErProRobot robot(ip, "192.168.0.180"); // 本机地址192.168.0.100

    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m, RtSupportedFields::jointVel_m, RtSupportedFields::tau_m});

    std::array<double,7> q_drag_xm7p = {0, 0, 0, 0, 0, 0, 0};

    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.5, robot.jointPos(ec), q_drag_xm7p);

    rtCon->startMove(RtControllerMode::jointPosition);

    //traj
    double rad_t = 0;
    array<double, 7> q_end;
    Matrix<double, 7, 11> coeffs;
    Matrix<double, 11, 1> fourier;
	Matrix<double, 7, 1> setpoint;   


    //读取txt中的轨迹信息
    ifstream file("/home/robot/robot/traj/opt_x.txt");
    int cnt = 0;
	std::string line_s;
	int row, column;

	while(getline(file, line_s)){
		row = round(cnt / 11);
		column = cnt % 11;
		coeffs(row, column) = double(atof(line_s.c_str()));
		cnt ++;
	}
    file.close();


	// Compute trajectory setpoints
	vector<array<double, 7>> joint_motion_vector;
	for (int cnt = 0; cnt <= CNT_PNTS; cnt++){
		rad_t = cnt * PERIOD * RAD_FREQ;
		fourier << sin(rad_t * 1) / (RAD_FREQ * 1), -cos(rad_t * 1) / (RAD_FREQ * 1),
				   sin(rad_t * 2) / (RAD_FREQ * 2), -cos(rad_t * 2) / (RAD_FREQ * 2),
				   sin(rad_t * 3) / (RAD_FREQ * 3), -cos(rad_t * 3) / (RAD_FREQ * 3), 
				   sin(rad_t * 4) / (RAD_FREQ * 4), -cos(rad_t * 4) / (RAD_FREQ * 4),
				   sin(rad_t * 5) / (RAD_FREQ * 5), -cos(rad_t * 5) / (RAD_FREQ * 5), 1;
		setpoint = coeffs * fourier;		// add scaling factor: 0.3
		q_end = {setpoint(0), setpoint(1), setpoint(2), setpoint(3), setpoint(4), setpoint(5), setpoint(6)};
		joint_motion_vector.push_back(q_end);
	}

    static bool init = true;
    double time = 0;
    array<double, 7> init_position;
	int vec_cnt = 0;	// counter for joint_motion_vector


    array<double,7> jntPos{}, jntVel{}, tau{};
    JointPosition cmd;
    function<JointPosition(void)> callback = [&, rtCon](){
        time += 0.001;
		// initialize
        if(init == true){
            init_position = robot.jointPos(ec);
            init = false;
        }

        jntPos = robot.jointPos(ec);
        jntVel = robot.jointVel(ec);
        tau = robot.jointTorque(ec);

        cout << jntPos << endl;
        position_file << "Position: " << jntPos << endl;
        velociy_file << "Velocity: " << jntVel << endl;
        torque_file << "Torque: " << tau << endl;

        // give cmd 
        cmd = {{joint_motion_vector[vec_cnt][0], joint_motion_vector[vec_cnt][1],
				   joint_motion_vector[vec_cnt][2], joint_motion_vector[vec_cnt][3],
				   joint_motion_vector[vec_cnt][4], joint_motion_vector[vec_cnt][5],
				   joint_motion_vector[vec_cnt][6]}};
        vec_cnt++;

		if ((time >= 20) || (vec_cnt >= (CNT_PNTS+1))) {
            cout << "<INFO> MOTION is over." << endl;
            cmd.setFinished();
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

    return 0;
}

