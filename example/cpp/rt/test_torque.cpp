#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "Eigen/Geometry"
#include "../print_helper.hpp"
#include "rokae/utility.h"
#include <fstream>



// using namespace std;
using namespace rokae;

void torqueControl(xMateErProRobot &robot) {
  using namespace RtSupportedFields;
  auto rtCon = robot.getRtMotionController().lock();
  auto model = robot.model();
  error_code ec;
  std::array<double,7> q_drag = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0 };

  robot.stopReceiveRobotState();
  robot.startReceiveRobotState(std::chrono::milliseconds(1),
                               {jointPos_m, jointVel_m, jointAcc_c, tcpPose_m});

  // 运动到拖拽位置
  rtCon->MoveJ(0.2, robot.jointPos(ec), q_drag);

  // 控制模式为力矩控制
  rtCon->startMove(RtControllerMode::torque);

  std::function<Torque(void)> callback = [&]{
    using namespace RtSupportedFields;
    static double time=0;
    time += 0.001;
    // 定义期望轨迹
    double delta_angle0 = M_PI / 20.0 * (1 - std::cos(M_PI * time));
    double delta_angle1 = M_PI / 20.0 * (1 - std::cos(M_PI / 2 * time));
    double delta_angle2 = M_PI / 20.0 * (1 - std::cos(M_PI * time));
    double delta_angle3 = M_PI / 20.0 * (1 - std::cos(M_PI / 2 * time));
    double delta_angle4 = M_PI / 20.0 * (1 - std::cos(M_PI / 4 * time));
    double delta_angle5 = M_PI / 20.0 * (1 - std::cos(M_PI / 2 * 3 * time));
    double delta_angle6 = M_PI / 20.0 * (1 - std::cos(M_PI / 4 * 3 * time));

    double dot_delta_angle0 = -M_PI / 20.0 * M_PI * std::sin(M_PI * time);
    double dot_delta_angle1 = -M_PI / 20.0 * M_PI / 2 * std::cos(M_PI / 2 * time);
    double dot_delta_angle2 = -M_PI / 20.0 * M_PI * std::cos(M_PI * time);
    double dot_delta_angle3 = -M_PI / 20.0 * M_PI / 2 * std::cos(M_PI / 2 * time);
    double dot_delta_angle4 = -M_PI / 20.0 * M_PI / 4 * std::cos(M_PI / 4 * time);
    double dot_delta_angle5 = -M_PI / 20.0 * M_PI / 2 * 3 * std::cos(M_PI / 2 * 3 * time);
    double dot_delta_angle6 = -M_PI / 20.0 * M_PI / 4 * 3 * std::cos(M_PI / 4 * 3 * time);

   if(time > 8){
     delta_angle0 = 0;
     delta_angle1 = 0;
     delta_angle2 = 0;
     delta_angle3 = 0;
     delta_angle4 = 0;
     delta_angle5 = 0;
     delta_angle6 = 0;
    }
    // 定义参数
    std::array<double, 7> error_q{}, error_dot_q{};//位置误差和速度误差
    std::array<double, 7> q{}, dot_q{}, dot_q_before{}, dot_dot_q{};//位置速度加速度
    std::array<double, 7> K{}, D{};//计算tau_fb
    std::array<double, 7> mu{};//不确定性
    std::array<double, 21> X{};//训练的输入
    std::array<double, 7> Y{};//训练的输出
    std::array<double, 7> tau_bar{}, tau_control{}, tau_fb{}, tau_ff{};//力矩信息
    std::array<double, 7> ine{}, cor{}, gra{};//机器人参数
    //读取tau_collect
    ifstream in("/home/robot/robot/roake_param_identify/build/torque_ly.txt",ios::in);
    string str;
    //解析txt文本
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
    vector<vector<double>> tau_collect(10001, vector<double>(7));
    int rows = 0; // mat行号
    int cols = 0; // mat列号
    string line;
    while (getline(file, line))
    {
        vector<string> tokens = split_func(line, ',');
        cols = 0;
        for (const auto &t : tokens)
        {
            tau_collect[rows][cols] = stod(t);
            cols++;
        }
        rows++;
    }
    
    // 位置速度信息
    q = robot.jointPos(ec);
    dot_q = robot.jointVel(ec);
    dot_q_before = 0;
    if(time >= 0.001){
      dot_q_before = dot_q;
    }
    dot_dot_q = (dot_q - dot_q_before) / 0.001;
    //误差
    error_q[0] = q[0] - delta_angle0;
    error_q[1] = q[1] - delta_angle1;
    error_q[2] = q[2] - delta_angle2;
    error_q[3] = q[3] - delta_angle3;
    error_q[4] = q[4] - delta_angle4;
    error_q[5] = q[5] - delta_angle5;
    error_q[6] = q[6] - delta_angle6;
    error_dot_q[0] = dot_q[0] - dot_delta_angle0;
    error_dot_q[1] = dot_q[1] - dot_delta_angle1;
    error_dot_q[2] = dot_q[2] - dot_delta_angle2;
    error_dot_q[3] = dot_q[3] - dot_delta_angle3;
    error_dot_q[4] = dot_q[4] - dot_delta_angle4;
    error_dot_q[5] = dot_q[5] - dot_delta_angle5;
    error_dot_q[6] = dot_q[6] - dot_delta_angle6;
    // 计算tau_fb
    K = 200 * Eigen::Matrix::Identity(7, 7);
    D = 6 * Eigen::Matrix::Identity(7, 7);
    tau_fb = - K * error_q - D * error_dot_q;
    // 训练mu
    X << q, dot_q, dot_dot_q;
    gra = model.getTorque(q, dot_q, dot_dot_q, Torquetype::gravity);
    tau_bar = gra;
    Y = tau_collect[1] - tau_bar;
    //这一行是计算mu 需要调用python函数
    mu[0] = 0;
    mu[1] = 1;
    mu[2] = 2;
    mu[3] = 3;
    mu[4] = 4;
    mu[5] = 5;
    mu[5] = 6;
    // 计算tau_ff
    tau_ff = gra + mu;
    // 计算tau_control
    tau_control = tau_fb + tau_ff;

    Torque cmd(7);
    Eigen::VectorXd::Map(cmd.tau.data(), 7) = tau_control;

    if(time > 10){
      cmd.setFinished();
    }
    return cmd;
  };

  // 由于需要在callback里读取状态数据, 这里useStateDataInLoop = true
  // 并且调用startReceiveRobotState()时, 设定的发送周期是1ms
  rtCon->setControlLoop(callback, 0, true);
  rtCon->startLoop(true);
  print(std::cout, "力矩控制结束");

}

/**
 * @brief 发送0力矩. 力控模型准确的情况下, 机械臂应保持静止不动
 */
template <unsigned short DoF>
void zeroTorque(Cobot<DoF> &robot) {
  std::vector<std::string> paths;

  error_code ec;
  std::array<double,7> q_drag = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
  array<double, 7> q_start = {0, 0, 0, 0, 0, 0, 0};
  auto rtCon = robot.getRtMotionController().lock();

  // 运动到拖拽位置
  rtCon->MoveJ(0.5, robot.jointPos(ec), q_drag);
  print(cout, "move done");


  // 控制模式为力矩控制
  rtCon->startMove(RtControllerMode::torque);
  Torque cmd {};
  cmd.tau.resize(DoF);
  char comm = ' ';

  std::function<Torque(void)> callback = [&]() {
    static double time=0;
    time += 0.001;
    if(cin.get() == 'q')
      cmd.setFinished();

    return cmd;
  };

  rtCon->setControlLoop(callback);
  rtCon->startLoop();
  print(std::cout, "力矩控制结束");
  rtCon->MoveJ(0.6, robot.jointPos(ec), q_start);
}

int main() {
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
