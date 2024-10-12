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
#include <sstream>
#include <string>

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
    Eigen::MatrixXd stiffness(7, 7), damping(7, 7);
    stiffness << -1500, 0, 0, 0, 0, 0, 0,
                 0, -1500, 0, 0, 0, 0, 0,
                 0, 0, -1500, 0, 0, 0, 0,
                 0, 0, 0, -1500, 0, 0, 0,
                 0, 0, 0, 0, -500, 0, 0,
                 0, 0, 0, 0, 0, -500, 0,
                 0, 0, 0, 0, 0, 0, -500;
    // stiffness << -2500, 0, 0, 0, 0, 0, 0,
    //              0, -2500, 0, 0, 0, 0, 0,
    //              0, 0, -12500, 0, 0, 0, 0,
    //              0, 0, 0, -2500, 0, 0, 0,
    //              0, 0, 0, 0, -1000, 0, 0,
    //              0, 0, 0, 0, 0, -1000, 0,
    //              0, 0, 0, 0, 0, 0, -1000;
    damping << -30, 0, 0, 0, 0, 0, 0,
               0, -30, 0, 0, 0, 0, 0,
               0, 0, -30, 0, 0, 0, 0,
               0, 0, 0, -30, 0, 0, 0,
               0, 0, 0, 0, -10, 0, 0,
               0, 0, 0, 0, 0, -10, 0,
               0, 0, 0, 0, 0, 0, -10;
    // damping << -50, 0, 0, 0, 0, 0, 0,
    //            0, -50, 0, 0, 0, 0, 0,
    //            0, 0, -50, 0, 0, 0, 0,
    //            0, 0, 0, -50, 0, 0, 0,
    //            0, 0, 0, 0, -30, 0, 0,
    //            0, 0, 0, 0, 0, -30, 0,
    //            0, 0, 0, 0, 0, 0, -30;
    
    //解析txt文本
    string str;
    auto split_func = [](const string &s, char delimiter)
        -> vector<string>
    {
        vector<string> tokens;
        string token;
        istringstream tokenStream(s);
        while (getline(tokenStream, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    };
    //
    //读取position
    // std::ifstream file_position("/home/robot/robot/roake_param_identify/build/ly_data/SGPR_cartesian1/collect/hat_p.txt");

    std::ifstream file_position("/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/collect/hat_p.txt");
    // std::ifstream file_position("/home/robot/robot/roake_param_identify/build/drag/drag_data/hat_p.txt");
    vector<vector<double>> collect_position(11077, vector<double>(7));//11077
    int rows = 0; // mat行号
    int cols = 0; // mat列号
    string line;
    while (getline(file_position, line))
    {
        vector<string> tokens = split_func(line, ' ');
        cols = 0;
        for (const auto &t : tokens)
        {
            collect_position[rows][cols] = stod(t);
            cols++;
        }
        rows++;
    }
    //读取velocity
    // std::ifstream file_velocity("/home/robot/robot/roake_param_identify/build/ly_data/SGPR_cartesian1/collect/hat_dp.txt");

    std::ifstream file_velocity("/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/collect/hat_dp.txt");
    // std::ifstream file_velocity("/home/robot/robot/roake_param_identify/build/drag/drag_data/hat_dp.txt");
    vector<vector<double>> collect_velocity(11076, vector<double>(7));//11076
    rows = 0;
    cols = 0;
    while (getline(file_position, line))
    {
        vector<string> tokens = split_func(line, ' ');
        cols = 0;
        for (const auto &t : tokens)
        {
            collect_velocity[rows][cols] = stod(t);
            cols++;
        }
        rows++;
    }
 
    std::array<double, 16> init_position {};
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), init_position);
    std::array<double, 7> q{}, dq_m{}, ddq_c{}, dq_before{}, tau{};
    for(int i=0; i<7; i++)
    {
        dq_before[i] = 0;
    }

    // ofstream position_file;
    ofstream position_error_file;
    // ofstream velocity_file;
    // ofstream acceleration_file;
    // ofstream inertia_file;
    // ofstream coriolis_file;
    // ofstream gravity_file;
    // ofstream torque_file;
    // // position_file.open("./drag/PD_error/position_ly.txt");
    position_error_file.open("/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/PD/position_error_ly.txt");
    // // velocity_file.open("./drag/PD_error/velocity_file.txt");
    // // acceleration_file.open("./drag/PD_error/acceleration_file.txt");
    // // inertia_file.open("./drag/PD_error/inertia_file.txt");
    // // coriolis_file.open("./drag/PD_error/coriolis_file.txt");
    // // gravity_file.open("./drag/PD_error/gravity_file.txt");
    // // torque_file.open("./drag/PD_error/torque_file.txt");
    
    // // position_file.open("./ly_data/SGPR/PD/position_ly.txt");
    // // position_error_file.open("./ly_data/SGPR/PD/position_error_ly.txt");
    // // velocity_file.open("./ly_data/SGPR/PD/velocity_file.txt");
    // // acceleration_file.open("./ly_data/SGPR/PD/acceleration_file.txt");
    // // inertia_file.open("./ly_data/SGPR/PD/inertia_file.txt");
    // // coriolis_file.open("./ly_data/SGPR/PD/coriolis_file.txt");
    // // gravity_file.open("./ly_data/SGPR/PD/gravity_file.txt");
    // // torque_file.open("./ly_data/SGPR/PD/torque_file.txt");

    // position_file.open("./ly_data/SGPR_cartesian/PD/position_ly.txt");
    // position_error_file.open("./ly_data/SGPR_cartesian/PD/position_error_ly.txt");
    // velocity_file.open("./ly_data/SGPR_cartesian/PD/velocity_file.txt");
    // acceleration_file.open("./ly_data/SGPR_cartesian/PD/acceleration_file.txt");
    // inertia_file.open("./ly_data/SGPR_cartesian/PD/inertia_file.txt");
    // coriolis_file.open("./ly_data/SGPR_cartesian/PD/coriolis_file.txt");
    // gravity_file.open("./ly_data/SGPR_cartesian/PD/gravity_file.txt");
    // torque_file.open("./ly_data/SGPR_cartesian/PD/torque_file.txt");


    // // position_file.open("./ly_data/SGPR_cartesian1/PD/position_ly.txt");
    // // position_error_file.open("./ly_data/SGPR_cartesian1/PD/position_error_ly.txt");
    // // velocity_file.open("./ly_data/SGPR_cartesian1/PD/velocity_file.txt");
    // // acceleration_file.open("./ly_data/SGPR_cartesian1/PD/acceleration_file.txt");
    // // inertia_file.open("./ly_data/SGPR_cartesian1/PD/inertia_file.txt");
    // // coriolis_file.open("./ly_data/SGPR_cartesian1/PD/coriolis_file.txt");
    // // gravity_file.open("./ly_data/SGPR_cartesian1/PD/gravity_file.txt");
    // // torque_file.open("./ly_data/SGPR_cartesian1/PD/torque_file.txt");

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
      //获取参考轨迹
      // '''参考位置'''
      static double times = 0;
      std::array<double, 7> angle{}, dot_angle{};
      for(int i=0;i<7;i++)
      {
        angle[i] = collect_position[times][i];
        dot_angle[i] = collect_velocity[times][i];
      } 
      times += 1;
      //计算误差
      // ''' 位置误差'''
      std::array<double, 7> error_jp{}, error_jv{};
      for(int i=0;i<7;i++)
      {
        error_jp[i] = q[i] - angle[i];
      }
      // cout << error_jp << endl;
      // '''速度误差'''
      for(int i=0;i<7;i++)
      {
        error_jv[i] = dq_m[i] - dot_angle[i];
      }
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
      cout << tau_d.transpose() << endl;

      Torque cmd(7);
      Eigen::VectorXd::Map(cmd.tau.data(), 7) = tau_d;

      // // 保存数据
      // position_file << q << endl;
      position_error_file << error_jp << endl;
      // velocity_file << dq_m << endl;
      // acceleration_file << ddq_c << endl;
      // inertia_file << ine << endl;
      // coriolis_file << cor << endl;
      // gravity_file << gra << endl;
      // torque_file << tau << endl;


      if(time > 60)
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

    // position_file.close();
    position_error_file.close();
    // velocity_file.close();
    // acceleration_file.close();
    // inertia_file.close();
    // coriolis_file.close();
    // gravity_file.close();
    // torque_file.close();

}

int main() 
{
  try 
  {
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
