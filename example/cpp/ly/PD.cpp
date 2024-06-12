#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <iostream>
#include <cmath>
#include <fstream>
#include <thread>
#include "Eigen/Geometry"
#include "rokae/utility.h"
#include <vector>
#include "../print_helper.hpp"
#include "rokae/robot.h"

#include <python3.8/Python.h>
#include </home/robot/.local/lib/python3.8/site-packages/numpy/core/include/numpy/arrayobject.h>

using namespace std;
using namespace rokae;
using namespace RtSupportedFields;

void torqueControl(xMateErProRobot &robot) 
{
    auto rtCon = robot.getRtMotionController().lock();
    auto model = robot.model();
    error_code ec;
    std::array<double,7> q_drag = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
    robot.stopReceiveRobotState();
    robot.startReceiveRobotState(std::chrono::milliseconds(1),
                                {jointPos_m, jointVel_m, jointAcc_c});
    // 运动到拖拽位置
    rtCon->MoveJ(0.2, robot.jointPos(ec), q_drag);
    // 控制模式为力矩控制
    rtCon->startMove(RtControllerMode::torque);

    // Compliance parameters
    const double translational_stiffness{-60.0};//15 and 12
    const double rotational_stiffness{5.0};
    Eigen::MatrixXd stiffness(7, 7), damping(7, 7);
    // stiffness.setZero();
    // stiffness.topLeftCorner(7, 7) << translational_stiffness * Eigen::MatrixXd::Identity(7, 7);
    // damping.setZero();
    // damping.topLeftCorner(7, 7) << translational_stiffness * Eigen::MatrixXd::Identity(7, 7);
    stiffness << -1000, 0, 0, 0, 0, 0, 0,
                 0, -1000, 0, 0, 0, 0, 0,
                 0, 0, -1000, 0, 0, 0, 0,
                 0, 0, 0, -1000, 0, 0, 0,
                 0, 0, 0, 0, -500, 0, 0,
                 0, 0, 0, 0, 0, -500, 0,
                 0, 0, 0, 0, 0, 0, -500;  
    damping << 0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0;
 
    std::array<double, 16> init_position {};
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), init_position);
    std::array<double, 7> q{}, dq_m{}, ddq_c{}, dq_before{}, tau{};
    for(int i=0; i<7; i++)
    {
        dq_before[i] = 0;
    }

    ofstream position_file;
    ofstream position_error_file;
    ofstream velocity_file;
    ofstream acceleration_file;
    ofstream inertia_file;
    ofstream coriolis_file;
    ofstream gravity_file;
    ofstream torque_file;
    position_file.open("./PD_error/position_ly.txt");
    position_error_file.open("./PD_error/position_error_ly.txt");
    velocity_file.open("./PD_error/velocity_file.txt");
    acceleration_file.open("./PD_error/acceleration_file.txt");
    inertia_file.open("./PD_error/inertia_file.txt");
    coriolis_file.open("./PD_error/coriolis_file.txt");
    gravity_file.open("./PD_error/gravity_file.txt");
    torque_file.open("./PD_error/torque_file.txt");
    
    std::function<Torque(void)> callback = [&]
    {
      static double time=0;
      time += 0.001;
      // 接收设置为true, 回调函数中可以直接读取
      q = robot.jointPos(ec);
      dq_m = robot.jointVel(ec);
      tau = robot.jointTorque(ec);

      for(int i=0; i<7; i++)
      {
          ddq_c[i] = (dq_m[i]-dq_before[i])/0.001;
      }
      dq_before = dq_m;
      //定义参考轨迹
      // '''参考位置'''
      double delta_angle0 = M_PI / 50.0 * (1 - std::cos(M_PI / 4 * 1 * time));
      double delta_angle1 = M_PI / 50.0 * (1 - std::cos(M_PI / 4 * 3 * time));
      double delta_angle2 = M_PI / 50.0 * (1 - std::cos(M_PI / 2 * 1 * time));
      double delta_angle3 = M_PI / 70.0 * (1 - std::cos(M_PI * time));
      double delta_angle4 = M_PI / 60.0 * (1 - std::cos(M_PI * time));
      double delta_angle5 = M_PI / 60.0 * (1 - std::cos(M_PI / 2 * 4 * time));
      double delta_angle6 = M_PI / 70.0 * (1 - std::cos(M_PI / 2 * 3 * time));
      // '''参考速度 '''
      double dot_delta_angle0 = M_PI / 50.0 * M_PI / 4 * 1 * std::sin(M_PI / 4 * 1 * time);
      double dot_delta_angle1 = M_PI / 50.0 * M_PI / 4 * 3 * std::sin(M_PI / 4 * 3 * time);
      double dot_delta_angle2 = M_PI / 50.0 * M_PI / 2 * 1 * std::sin(M_PI / 2 * 1 * time);
      double dot_delta_angle3 = M_PI / 70.0 * M_PI * std::sin(M_PI * time);
      double dot_delta_angle4 = M_PI / 60.0 * M_PI * std::sin(M_PI * time);
      double dot_delta_angle5 = M_PI / 60.0 * M_PI / 2 * 4 * std::sin(M_PI / 2 * 4 * time);
      double dot_delta_angle6 = M_PI / 70.0 * M_PI / 2 * 3 * std::sin(M_PI / 2 * 3 * time);
      if(time > 8)
      {
      delta_angle0 = 0;
      delta_angle1 = 0;
      delta_angle2 = 0;
      delta_angle3 = 0;
      delta_angle4 = 0;
      delta_angle5 = 0;
      delta_angle6 = 0;
      dot_delta_angle0 = 0;
      dot_delta_angle1 = 0;
      dot_delta_angle2 = 0;
      dot_delta_angle3 = 0;
      dot_delta_angle4 = 0;
      dot_delta_angle5 = 0;
      dot_delta_angle6 = 0;
      }
      //计算误差
      // ''' 位置误差'''
      std::array<double, 7> error_jp{}, error_jv{};
      error_jp[0] = q[0] - delta_angle0;
      error_jp[1] = q[1] - delta_angle1 - M_PI/6;
      error_jp[2] = q[2] - delta_angle2;
      error_jp[3] = q[3] - delta_angle3 - M_PI/3;
      error_jp[4] = q[4] - delta_angle4;
      error_jp[5] = q[5] - delta_angle5 - M_PI/2;
      error_jp[6] = q[6] - delta_angle6;
      // cout << error_jp << endl;
      // '''速度误差'''
      error_jv[0] = dq_m[0] - dot_delta_angle0;
      error_jv[1] = dq_m[1] - dot_delta_angle1;
      error_jv[2] = dq_m[2] - dot_delta_angle2;
      error_jv[3] = dq_m[3] - dot_delta_angle3;
      error_jv[4] = dq_m[4] - dot_delta_angle4;
      error_jv[5] = dq_m[5] - dot_delta_angle5;
      error_jv[6] = dq_m[6] - dot_delta_angle6;
      // 获取各项力
      std::array<double, 7> ine = model.getTorque(q, dq_m, ddq_c, TorqueType::inertia);
      std::array<double, 7> cor = model.getTorque(q, dq_m, ddq_c, TorqueType::coriolis);
      std::array<double, 7> gra = model.getTorque(q, dq_m, ddq_c, TorqueType::gravity);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> error_jp_mat(error_jp.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> error_jv_mat(error_jv.data());
      // compute control
      Eigen::VectorXd tau_ff(7);
      tau_ff = stiffness * error_jp_mat + damping * error_jv_mat;
      Eigen::VectorXd tau_d(7);
      tau_d = tau_ff;

      Torque cmd(7);
      Eigen::VectorXd::Map(cmd.tau.data(), 7) = tau_d;

      // 保存数据
      position_file << q << endl;
      position_error_file << error_jp << endl;
      velocity_file << dq_m << endl;
      acceleration_file << ddq_c << endl;
      inertia_file << ine << endl;
      coriolis_file << cor << endl;
      gravity_file << gra << endl;
      torque_file << tau << endl;


      if(time > 10)
      {
      cmd.setFinished();
      }
      return cmd;
    };
    // 由于需要在callback里读取状态数据, 这里useStateDataInLoop = true
    // 并且调用startReceiveRobotState()时, 设定的发送周期是1ms
    rtCon->setControlLoop(callback, 0, true);
    rtCon->startLoop(true);
    print(std::cout, "力矩控制结束");

    position_file.close();
    position_error_file.close();
    velocity_file.close();
    acceleration_file.close();
    inertia_file.close();
    coriolis_file.close();
    gravity_file.close();
    torque_file.close();

}

int main() 
{
  
  try {
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.180"); // ****   xMate 7-axis
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);
    
    try {
      torqueControl(robot);
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
