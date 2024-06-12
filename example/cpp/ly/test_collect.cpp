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
  ofstream inertia_file;
  ofstream coriolis_file;
  ofstream gravity_file;
  ofstream torque_file;
  position_file.open("./collect/position.txt");
  velocity_file.open("./collect/velocity.txt");
  inertia_file.open("./collect/inertia.txt");
  coriolis_file.open("./collect/coriolis.txt");
  gravity_file.open("./collect/gravity.txt");
  torque_file.open("./collect/torque.txt");

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
    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
    // std::array<double,7> q_drag_xm7p = {0, 0, 0, 0, 0, 0, 0};
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
      // robot.getStateData(RtSupportedFields::tau_m, tau);

      jp = robot.jointPos(ec);
      jv = robot.jointVel(ec);
      tor = robot.jointTorque(ec);

      std::array<double, 7> inertia_array = model.getTorque(q, dq_m, ddq_c, TorqueType::inertia);
      std::array<double, 7> coriolis_array = model.getTorque(q, dq_m, ddq_c, TorqueType::coriolis);
      std::array<double, 7> gravity_array = model.getTorque(q, dq_m, ddq_c, TorqueType::gravity);

      double delta_angle0 = M_PI / 50.0 * (1 - std::cos(M_PI / 4 * 1 * time));
      double delta_angle1 = M_PI / 50.0 * (1 - std::cos(M_PI / 4 * 3 * time));
      double delta_angle2 = M_PI / 50.0 * (1 - std::cos(M_PI / 2 * 1 * time));
      double delta_angle3 = M_PI / 70.0 * (1 - std::cos(M_PI * time));
      double delta_angle5 = M_PI / 60.0 * (1 - std::cos(M_PI * time));
      double delta_angle4 = M_PI / 60.0 * (1 - std::cos(M_PI / 2 * 4 * time));
      double delta_angle6 = M_PI / 70.0 * (1 - std::cos(M_PI / 2 * 3 * time));

      if(time > 8)
      {
        delta_angle0 = 0;
        delta_angle1 = 0;
        delta_angle2 = 0;
        delta_angle3 = 0;
        delta_angle4 = 0;
        delta_angle5 = 0;
        delta_angle6 = 0;
      }

      JointPosition cmd = {{jntPos[0] + delta_angle0, jntPos[1] + delta_angle1,
                            jntPos[2] + delta_angle2, jntPos[3] + delta_angle3,
                            jntPos[4] + delta_angle4, jntPos[5] + delta_angle5,
                            jntPos[6] + delta_angle6}};
      // 保存数据
      inertia_file << inertia_array << endl;
      coriolis_file << coriolis_array << endl;
      gravity_file << gravity_array << endl;
      position_file << jp << endl;
      velocity_file << jv << endl;
      torque_file << tor << endl;
    
      if(time > 10)
        cmd.setFinished(); // 60秒后结束
      return cmd;
    };

    rtCon->setControlLoop(callback);
    // 阻塞loop
    rtCon->startLoop(true);
    print(std::cout, "控制结束");

    inertia_file.close();
    coriolis_file.close();
    gravity_file.close();
    position_file.close();
    velocity_file.close();
    torque_file.close();


  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}
