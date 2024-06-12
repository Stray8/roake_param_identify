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
    ext_force_file.open("externel_force.txt");
    cart_position_file.open("cart_position.txt");
    std::string ip = "192.168.0.160";  
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.180"); 
    // robot.setRtNetworkTolerance(100, ec);
    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);


    auto rtCon = robot.getRtMotionController().lock();

    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tauExt_inBase});

    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0 };
    rtCon->MoveJ(0.2, robot.jointPos(ec), q_drag_xm7p);
    // 设置力控坐标系为工具坐标系, 末端相对法兰的坐标系
    std::array<double, 16> toolToFlange = {0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1};
    rtCon->setFcCoor(toolToFlange, ForceControlFrameType::tool, ec);
    // 设置笛卡尔阻抗系数
    rtCon->setCartesianImpedance({1500, 1500, 0, 0, 0, 0}, ec);
    // 设置X和Z方向3N的期望力
    rtCon->setCartesianImpedanceDesiredTorque({0, 0, 0, 0, 0, 0}, ec);
    //获取当前机器人笛卡尔空间坐标

    std::array<double, 16> init_position{}, real_position{};
    array<double, 6> exterl_force{};

    Utils::postureToTransArray(robot.posture(CoordinateType::flangeInBase, ec), init_position);

    rtCon->startMove(RtControllerMode::cartesianImpedance);

    double time = 0;
    // std::atomic<bool> stopManually {true};
    std::function<CartesianPosition(void)> callback = [&, rtCon]()->CartesianPosition{
      time += 0.001;

      robot.getStateData(RtSupportedFields::tauExt_inBase, exterl_force);

      
      constexpr double kRadius = 0.2;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * time));
      double delta_z = kRadius * (std::cos(angle) - 1);

      CartesianPosition output{};
      

      output.pos = init_position;
      // output.pos[3] += delta_z;
      // output.pos[7] += delta_z;

      output.pos[11] += delta_z;

      Utils::postureToTransArray(robot.posture(CoordinateType::flangeInBase, ec), real_position);
      Eigen::Vector3d pos_1(real_position[3], real_position[7], real_position[11]);
      Eigen::Vector3d force(exterl_force[0],exterl_force[1],exterl_force[2]);

      cout << time << endl;
      cout << "Pos: " << pos_1.transpose() << endl;
      cout << "Force: " << force.transpose() << endl;
      
      ext_force_file  << force.transpose() << endl;
      cart_position_file << pos_1.transpose() << endl;

      if(time > 20){
        // std::cout << "mode done" <<std::endl;
        output.setFinished();
        // stopManually.store(false); // loop为非阻塞，和主线程同步停止状态
      }
      return output;
    };
    rtCon->setControlLoop(callback);
    rtCon->startLoop(true);
    // while(stopManually.load());
    // rtCon->stopLoop();
    cart_position_file.close();
    ext_force_file.close();
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    rtCon->MoveJ(0.2, robot.jointPos(ec), q_drag_xm7p);

    return 0;
}

