#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include <fstream>


using namespace rokae;
using namespace std;
using namespace Eigen;


int main(){
    ofstream cart_position_file;
    ofstream ext_force_file;
    ofstream ext_force_base_file;
    ext_force_base_file.open("ext_force_base.txt");
    cart_position_file.open("cart_position.txt");
    std::string ip = "192.168.0.160";  
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.180"); 
    // robot.setRtNetworkTolerance(100, ec);
    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);


    auto rtCon = robot.getRtMotionController().lock();

    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tcpPose_m,
                                                                RtSupportedFields::tcpPoseAbc_m,
                                                                RtSupportedFields::tauExt_inBase});

    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0 };
    rtCon->MoveJ(0.2, robot.jointPos(ec), q_drag_xm7p);
    // 设置力控坐标系为工具坐标系, 末端相对法兰的坐标系
    std::array<double, 16> toolToFlange = {0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1};
    rtCon->setFcCoor(toolToFlange, ForceControlFrameType::tool, ec);
    // 设置笛卡尔阻抗系数
    rtCon->setCartesianImpedance({1500, 1500, 1000, 100, 100, 100}, ec);
    // 设置X和Z方向3N的期望力
    rtCon->setCartesianImpedanceDesiredTorque({3, 0, 3, 0, 0, 0}, ec);
    //获取当前机器人笛卡尔空间坐标
    
    //16个数据是4*4的矩阵
    // r11,r12,r13,tx
    // r21,r22,r23,ty
    // r31,r32,r33,tz
    // 0,  0,  0,  1

    std::array<double, 16> init_position{}, real_position{};
    array<double, 6> exterl_force{}, ext_force_base{}, cart_pose{};

    robot.getStateData(RtSupportedFields::tcpPose_m, init_position);

    rtCon->startMove(RtControllerMode::cartesianImpedance);

    double time = 0;
    // std::atomic<bool> stopManually {true};
    std::function<CartesianPosition(void)> callback = [&, rtCon]()->CartesianPosition{
      time += 0.001;

      robot.getStateData(RtSupportedFields::tcpPoseAbc_m, cart_pose);
      robot.getStateData(RtSupportedFields::tauExt_inBase, ext_force_base);

      
      constexpr double kRadius = 0.2;
      double delta_z = kRadius * (cos(time) - 1);


      CartesianPosition output{};
      output.pos = init_position;

      Eigen::Vector3d pos_1(cart_pose[0], cart_pose[1], cart_pose[2]);
      Eigen::Vector3d ext_force_base(ext_force_base[0],ext_force_base[1],ext_force_base[2]);

      cout << time << endl;
      cout << "Pos: " << pos_1.transpose() << endl;
      cout << "Force_bae: " << ext_force_base.transpose() << endl;

      ext_force_base_file << ext_force_base.transpose() << endl;      
      cart_position_file << pos_1.transpose() << endl;

      if(time > 20){
        // std::cout << "mode done" <<std::endl;
        output.setFinished();
        // stopManually.store(false); // loop为非阻塞，和主线程同步停止状态
      }
      return output;
    };
    rtCon->setControlLoop(callback, 0, true);
    rtCon->startLoop(true);
    // while(stopManually.load());
    // rtCon->stopLoop();
    cart_position_file.close();
    ext_force_base_file.close();
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    rtCon->MoveJ(0.4, robot.jointPos(ec), q_drag_xm7p);

    return 0;
}

