#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "Eigen/Geometry"
#include "../print_helper.hpp"
#include <fstream>
#include <time.h>
extern "C"{
    #include "/home/robot/robot/roake_param_identify/include/robotiq/rq_sensor_com.h"
    #include "/home/robot/robot/roake_param_identify/include/robotiq/rq_sensor_state.h"
}

using namespace rokae;
using namespace std;
using namespace RtSupportedFields;


static void wait_for_other_connection(){
	struct timespec tim;
	tim.tv_sec = 1;
	tim.tv_nsec = 0L;

	while(1){
		// nanosleep(&tim, (struct timespec *)NULL);
		if(rq_sensor_state == 0){
			break;
		}
	}
}


static void get_data(char* chr_return){
    int i;
    for (i = 0; i < 6; i++){
        printf("%f ", rq_state_get_received_data(i));
        if(i == 5){
            printf("\n");
        }
    }
}

void cart_control(xMateErProRobot &robot) {
    
}

int main(){
    ofstream cart_posx_file;  
    ofstream cart_posy_file;    
    ofstream cart_force_z;
    cart_posx_file.open("cart_posx.txt");
    cart_posy_file.open("cart_posy.txt");
    cart_force_z.open("cart_force_z.txt");
    char ret = rq_sensor_state();
    if(ret == -1){
        wait_for_other_connection();
    }
	ret = rq_sensor_state();
	if(ret == -1){
		wait_for_other_connection();
	}
	ret = rq_sensor_state();
	if(ret == -1){
		wait_for_other_connection();
	}
    char bufStream[512];
    cout << "start" << endl;

    try {
        std::string ip = "192.168.0.160";
        std::error_code ec;
        rokae::xMateErProRobot robot(ip, "192.168.0.180"); // ****   xMate 7-axis
        robot.setRtNetworkTolerance(50, ec);
        robot.setOperateMode(rokae::OperateMode::automatic,ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);
        try{
            std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0 };
            auto rtCon = robot.getRtMotionController().lock();

            rtCon->setFilterLimit(true, 10);
            rtCon->setFilterFrequency(10, 10, 10, ec);
            
            robot.stopReceiveRobotState();
            robot.startReceiveRobotState(std::chrono::milliseconds(2),{jointPos_m, 
                                                                        jointVel_m, 
                                                                        jointAcc_c, 
                                                                        tcpPose_m,
                                                                        tcpPoseAbc_m, 
                                                                        tauExt_inBase,
                                                                        tauExt_inStiff});
            rtCon->MoveJ(0.4, robot.jointPos(ec), q_drag_xm7p);
            cout << "move done!" << endl;

            // 设置力控坐标系为工具坐标系, 末端相对法兰的坐标系
            std::array<double, 16> toolToFlange = {0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1};
            rtCon->setFcCoor(toolToFlange, ForceControlFrameType::tool, ec);
            
            std::array<double, 16> init_position{};
            robot.getStateData(tcpPose_m, init_position);
            rtCon->startMove(RtControllerMode::cartesianPosition);
            array<double,6> cart_pos{};
            double time = 0;
            double f_ext = 1;
            double f_true;
            std::function<CartesianPosition(void)> callback = [&, rtCon]()->CartesianPosition{
                time += 0.002;
                rq_sensor_state();
                f_true = rq_state_get_received_data(2);
                robot.getStateData(tcpPoseAbc_m, cart_pos);
                /*
                走圆计算
                (x - 起始x位置)^2 + (y - 起始y位置)^2 = r^2
                x = rcos() + 起始x位置
                y = rsin() + 起始y位置
                起始x位置 + x增量 = x 反求x增量 直接给x位置会报错
                */

                constexpr double kRadius = 0.1;
                double delta_x = kRadius * std::cos(M_PI  * time);
                double delta_y = kRadius * std::sin(M_PI  * time);
                double delta_z = 1/5 * (f_ext - f_true);

                CartesianPosition output{};
                output.pos = init_position;
                output.pos[3] += delta_x - 0.1;
                output.pos[7] += delta_y;
                output.pos[11] += delta_z;


                //保存坐标
                cart_posx_file << cart_pos[0] << endl;
                cart_posy_file << cart_pos[1] << endl;
                cart_force_z << f_true << endl;
                cout << "time: " << time << endl;
                cout << "position: " << cart_pos << endl;
                cout << "torque z:" << f_true << endl;


                if(time > 4){
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
            cart_force_z.close();
        }
        catch (const rokae::RealtimeMotionException &e) {
            print(std::cerr, e.what());
            // 发生错误, 切换回非实时模式
            robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        }

        robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        robot.setOperateMode(rokae::OperateMode::manual, ec);
    } 
    catch (const std::exception &e) {
        print(std::cerr, e.what());
    }
    return 0;
}