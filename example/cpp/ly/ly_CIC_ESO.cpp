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

// #include <python3.8/Python.h>
// #include </home/robot/.local/lib/python3.8/site-packages/numpy/core/include/numpy/arrayobject.h>

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
    damping << -30, 0, 0, 0, 0, 0, 0,
               0, -30, 0, 0, 0, 0, 0,
               0, 0, -30, 0, 0, 0, 0,
               0, 0, 0, -30, 0, 0, 0,
               0, 0, 0, 0, -30, 0, 0,
               0, 0, 0, 0, 0, -30, 0,
               0, 0, 0, 0, 0, 0, -30;
 
    std::array<double, 16> init_position {};
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), init_position);
    std::array<double, 7> q{}, dq_m{}, ddq_c{}, dq_before{}, tau{};
    for(int i=0; i<7; i++)
    {
        dq_before[i] = 0;
    }

    std::array<double, 10001> qd0, qd1, qd2, qd3, qd4, qd5, qd6;
    std::array<double, 10001> dqd0, dqd1, dqd2, dqd3, dqd4, dqd5, dqd6;
    std::array<double, 10001> ddqd0, ddqd1, ddqd2, ddqd3, ddqd4, ddqd5, ddqd6;
    std::array<double, 10001> qr0, qr1, qr2, qr3, qr4, qr5, qr6;
    std::array<double, 10001> dqr0, dqr1, dqr2, dqr3, dqr4, dqr5, dqr6;
    std::array<double, 10001> ddqr0, ddqr1, ddqr2, ddqr3, ddqr4, ddqr5, ddqr6;
    std::array<double, 10001> f0, f1, f2, f3, f4, f5, f6;
    std::array<double, 10001> e0, e1, e2, e3, e4, e5, e6;
    std::array<double, 10001> de0, de1, de2, de3, de4, de5, de6;
    double m, d, k;
    m = 1;
    d = 10;
    k = 1000;
    
    std::function<Torque(void)> callback = [&]
    {
        static double time=0;
        static double times=0;
        
        // 接收设置为true, 回调函数中可以直接读取
        q = robot.jointPos(ec);
        dq_m = robot.jointVel(ec);
        tau = robot.jointTorque(ec);
        for(int i=0; i<7; i++)
        {
            ddq_c[i] = (dq_m[i]-dq_before[i])/0.001;
        }
        dq_before = dq_m;
        // 获取各项力
        std::array<double, 7> ine = model.getTorque(q, dq_m, ddq_c, TorqueType::inertia);
        std::array<double, 7> cor = model.getTorque(q, dq_m, ddq_c, TorqueType::coriolis);
        std::array<double, 7> gra = model.getTorque(q, dq_m, ddq_c, TorqueType::gravity);
        //定义参考轨迹
        if(times>1500)
        {
            f0[times] = 0;
            f1[times] = 3;
            f2[times] = 6;
            f3[times] = 9;
            f4[times] = 12;
            f5[times] = 15;
            f6[times] = 18;
        }
        qd0[times] = M_PI / 50.0 * (1 - std::cos(M_PI / 4 * 1 * time));
        qd1[times] = M_PI / 50.0 * (1 - std::cos(M_PI / 4 * 3 * time));
        qd2[times] = M_PI / 50.0 * (1 - std::cos(M_PI / 2 * 1 * time));
        qd3[times] = M_PI / 70.0 * (1 - std::cos(M_PI * time));
        qd4[times] = M_PI / 60.0 * (1 - std::cos(M_PI * time));
        qd5[times] = M_PI / 60.0 * (1 - std::cos(M_PI / 2 * 4 * time));
        qd6[times] = M_PI / 70.0 * (1 - std::cos(M_PI / 2 * 3 * time));
        dqd0[times] = M_PI / 50.0 * M_PI / 4 * 1 * std::sin(M_PI / 4 * 1 * time);
        dqd1[times] = M_PI / 50.0 * M_PI / 4 * 3 * std::sin(M_PI / 4 * 3 * time);
        dqd2[times] = M_PI / 50.0 * M_PI / 2 * 1 * std::sin(M_PI / 2 * 1 * time);
        dqd3[times] = M_PI / 70.0 * M_PI * std::sin(M_PI * time);
        dqd4[times] = M_PI / 60.0 * M_PI * std::sin(M_PI * time);
        dqd5[times] = M_PI / 60.0 * M_PI / 2 * 4 * std::sin(M_PI / 2 * 4 * time);
        dqd6[times] = M_PI / 70.0 * M_PI / 2 * 3 * std::sin(M_PI / 2 * 3 * time);
        if(time>8)
        {
            qd0[times] = 0;
            qd1[times] = 0;
            qd2[times] = 0;
            qd3[times] = 0;
            qd4[times] = 0;
            qd5[times] = 0;
            qd6[times] = 0;

            dqd0[times] = 0;
            dqd1[times] = 0;
            dqd2[times] = 0;
            dqd3[times] = 0;
            dqd4[times] = 0;
            dqd5[times] = 0;
            dqd6[times] = 0;
        }
        e0[times] = qr0[times] - qd0[times];
        e1[times] = qr1[times] - qd1[times];
        e2[times] = qr2[times] - qd2[times];
        e3[times] = qr3[times] - qd3[times];
        e4[times] = qr4[times] - qd4[times];
        e5[times] = qr5[times] - qd5[times];
        e6[times] = qr6[times] - qd6[times];

        de0[times] = dqr0[times] - dqd0[times];
        de1[times] = dqr1[times] - dqd1[times];
        de2[times] = dqr2[times] - dqd2[times];
        de3[times] = dqr3[times] - dqd3[times];
        de4[times] = dqr4[times] - dqd4[times];
        de5[times] = dqr5[times] - dqd5[times];
        de6[times] = dqr6[times] - dqd6[times];

        ddqr0[times+1] = (1/m)*f0[times] - (d/m)*de0[times] - (k/m)*e0[times];
        ddqr1[times+1] = (1/m)*f1[times] - (d/m)*de1[times] - (k/m)*e1[times];
        ddqr2[times+1] = (1/m)*f2[times] - (d/m)*de2[times] - (k/m)*e2[times];
        ddqr3[times+1] = (1/m)*f3[times] - (d/m)*de3[times] - (k/m)*e3[times];
        ddqr4[times+1] = (1/m)*f4[times] - (d/m)*de4[times] - (k/m)*e4[times];
        ddqr5[times+1] = (1/m)*f5[times] - (d/m)*de5[times] - (k/m)*e5[times];
        ddqr6[times+1] = (1/m)*f6[times] - (d/m)*de6[times] - (k/m)*e6[times];

        dqr0[times+1] = dqr0[times] + ddqr0[times+1] * 0.001;
        dqr1[times+1] = dqr1[times] + ddqr1[times+1] * 0.001;
        dqr2[times+1] = dqr2[times] + ddqr2[times+1] * 0.001;
        dqr3[times+1] = dqr3[times] + ddqr3[times+1] * 0.001;
        dqr4[times+1] = dqr4[times] + ddqr4[times+1] * 0.001;
        dqr5[times+1] = dqr5[times] + ddqr5[times+1] * 0.001;
        dqr6[times+1] = dqr6[times] + ddqr6[times+1] * 0.001;

        qr0[times+1] = qr0[times] + dqr0[times+1] * 0.001;
        qr1[times+1] = qr1[times] + dqr1[times+1] * 0.001;
        qr2[times+1] = qr2[times] + dqr2[times+1] * 0.001;
        qr3[times+1] = qr3[times] + dqr3[times+1] * 0.001;
        qr4[times+1] = qr4[times] + dqr4[times+1] * 0.001;
        qr5[times+1] = qr5[times] + dqr5[times+1] * 0.001;
        qr6[times+1] = qr6[times] + dqr6[times+1] * 0.001;
        //计算误差
        std::array<double, 7> ep, dep;
        ep[0] = qr0[times] - q[0];
        ep[1] = qr1[times] - q[1];
        ep[2] = qr2[times] - q[2];
        ep[3] = qr3[times] - q[3];
        ep[4] = qr4[times] - q[4];
        ep[5] = qr5[times] - q[5];
        ep[6] = qr6[times] - q[6];

        dep[0] = dqr0[times] - dq_m[0];
        dep[1] = dqr1[times] - dq_m[1];
        dep[2] = dqr2[times] - dq_m[2];
        dep[3] = dqr3[times] - dq_m[3];
        dep[4] = dqr4[times] - dq_m[4];
        dep[5] = dqr5[times] - dq_m[5];
        dep[6] = dqr6[times] - dq_m[6];
        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> ep_mat(ep.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dep_mat(dep.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> ine_mat(ine.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> cor_mat(cor.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> gra_mat(gra.data());
        // compute control
        Eigen::VectorXd tau_d(7);
        tau_d = stiffness * ep_mat + damping * dep_mat + ine_mat + cor_mat + gra_mat;

        Torque cmd(7);
        Eigen::VectorXd::Map(cmd.tau.data(), 7) = tau_d;
        cout << tau_d << endl;

        time += 0.001;
        times += 1;

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
