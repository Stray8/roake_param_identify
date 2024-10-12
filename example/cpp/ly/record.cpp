#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "Eigen/Geometry"
#include "../print_helper.hpp"
#include "rokae/utility.h"

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <fstream>
#include <vector>

using namespace std;
using namespace rokae;

template <unsigned short DoF>
void zeroTorque(Cobot<DoF> &robot) 
{
    using namespace RtSupportedFields;
    robot.stopReceiveRobotState();
    robot.startReceiveRobotState(std::chrono::milliseconds(1),
                                {jointPos_m, jointVel_m, jointAcc_c, tcpPose_m});
    std::vector<std::string> paths;
    error_code ec;
    std::array<double,7> q_drag = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
    array<double, 7> q_start = {0, 0, 0, 0, 0, 0, 0};
    auto rtCon = robot.getRtMotionController().lock();
    auto model = robot.model();
    // 运动到拖拽位置
    rtCon->MoveJ(0.5, robot.jointPos(ec), q_drag);
    print(cout, "move done");
    // 控制模式为力矩控制
    rtCon->startMove(RtControllerMode::torque);
    Torque cmd {};
    cmd.tau.resize(DoF);
    std::array<double, 7> q{}, dq{}, ddq{}, dq_before{}, ine{}, cor{}, gra{}, tor{}, ddq_c{}, q_m{}, dq_m{};
    for(int i=0;i<7;i++)
    {
      dq_before[i] = 0;
    }

    ofstream position;
    ofstream velocity;
    ofstream inertia;
    ofstream coriolis;
    ofstream gravity;
    ofstream torque;
    // position.open("./drag/drag_data/position.txt");
    // velocity.open("./drag/drag_data/velocity.txt");
    // inertia.open("./drag/drag_data/inertia.txt");
    // coriolis.open("./drag/drag_data/coriolis.txt");
    // gravity.open("./drag/drag_data/gravity.txt");
    // torque.open("./drag/drag_data/torque.txt");

    // position.open("./ly_data/SGPR_cartesian1/collect/position.txt");
    // velocity.open("./ly_data/SGPR_cartesian1/collect/velocity.txt");
    // inertia.open("./ly_data/SGPR_cartesian1/collect/inertia.txt");
    // coriolis.open("./ly_data/SGPR_cartesian1/collect/coriolis.txt");
    // gravity.open("./ly_data/SGPR_cartesian1/collect/gravity.txt");
    // torque.open("./ly_data/SGPR_cartesian1/collect/torque.txt");

    position.open("./ly_data/show/collect/position.txt");
    velocity.open("./ly_data/show/collect/velocity.txt");
    inertia.open("./ly_data/show/collect/inertia.txt");
    coriolis.open("./ly_data/show/collect/coriolis.txt");
    gravity.open("./ly_data/show/collect/gravity.txt");
    torque.open("./ly_data/show/collect/torque.txt");

    std::function<Torque(void)> callback = [&]() 
    {
      static double time=0;
      time += 0.001;
      tor = robot.jointTorque(ec);
      q = robot.jointPos(ec);
      dq = robot.jointVel(ec);
      for(int i=0;i<7;i++)
      {
        ddq[i] = (dq[i]-dq_before[i])/0.001;
      }
      dq_before = dq;

      // robot.getStateData(RtSupportedFields::jointPos_m, q_m);
      // robot.getStateData(RtSupportedFields::jointVel_m, dq_m);
      // robot.getStateData(RtSupportedFields::jointAcc_c, ddq_c);
      // cout << ddq_c << endl;

      std::array<double, 7> gra = model.getTorque(q, dq, ddq, TorqueType::gravity);
      std::array<double, 7> cor = model.getTorque(q, dq, ddq, TorqueType::coriolis);
      std::array<double, 7> ine = model.getTorque(q, dq, ddq, TorqueType::inertia);
      // // 保存数据
      position << q << endl;
      velocity << dq << endl;
      inertia << ine << endl;
      coriolis << cor << endl;
      gravity << gra << endl;
      torque << tor << endl;

      if(time > 30)
        cmd.setFinished();
      return cmd;
    };

    rtCon->setControlLoop(callback, 0, true);
    rtCon->startLoop();
    print(std::cout, "力矩控制结束");
    rtCon->MoveJ(0.6, robot.jointPos(ec), q_start);

    position.close();
    velocity.close();
    inertia.close();
    coriolis.close();
    gravity.close();
    torque.close();
}

int main(){
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.180"); // ****   xMate 7-axis
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);
    auto rtCon = robot.getRtMotionController().lock();
    try {
      zeroTorque(robot);
    } catch (const rokae::RealtimeMotionException &e) {
      print(std::cerr, e.what());
      // 发生错误, 切换回非实时模式
      robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    }

    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec);

  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}
