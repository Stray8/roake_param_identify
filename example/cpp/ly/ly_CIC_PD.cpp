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

    std::array<double, 7> qr, dqr, ddqr;
    for(int i=0;i<7;i++)
    {
        qr[i] = 0;
        dqr[i] = 0;
        ddqr[i] = 0;
    } 
    std::array<double, 7> z1, dz1, z2, dz2, z3, dz3;
    for(int i=0;i<7;i++)
    {
      z1[i] = 0;
      dz1[i] = 0;
      z2[i] = 0;
      dz2[i] = 0;
      z3[i] = 0;
      dz3[i] = 0;
    }

    ofstream q_file;
    ofstream dq_file;
    ofstream qr_file;
    ofstream z1_file;
    ofstream z2_file;
    ofstream z3_file;

    q_file.open("./ly_data/CIC/PD/q.txt");
    dq_file.open("./ly_data/CIC/PD/dq.txt");
    qr_file.open("./ly_data/CIC/PD/qr.txt");
    z1_file.open("./ly_data/CIC/PD/z1.txt");
    z2_file.open("./ly_data/CIC/PD/z2.txt");
    z3_file.open("./ly_data/CIC/PD/z3.txt");


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
      //接触力
      std::array<double, 7> dis;
      for(int i=0;i<7;i++)
      {
        dis[i] = 2;
      }
      //阻抗参数
      std::array<double, 7> m, d, k;
      for(int i=0;i<7;i++)
      {
        m[i] = 1;
        d[i] = 5;
        k[i] = 100;
      }
      //计算阻抗修正轨迹qr
      for(int i=0;i<7;i++)
      {
        ddqr[0] = (1/m[0])*dis[0] - (d[0]/m[0])*(dqr[0]-dot_delta_angle0) - (k[0]/m[0])*(qr[0]-delta_angle0);
        ddqr[1] = (1/m[1])*dis[1] - (d[1]/m[1])*(dqr[1]-dot_delta_angle1) - (k[1]/m[1])*(qr[1]-delta_angle1);
        ddqr[2] = (1/m[2])*dis[2] - (d[2]/m[2])*(dqr[2]-dot_delta_angle2) - (k[2]/m[2])*(qr[2]-delta_angle2);
        ddqr[3] = (1/m[3])*dis[3] - (d[3]/m[3])*(dqr[3]-dot_delta_angle3) - (k[3]/m[3])*(qr[3]-delta_angle3);
        ddqr[4] = (1/m[4])*dis[4] - (d[4]/m[4])*(dqr[4]-dot_delta_angle4) - (k[4]/m[4])*(qr[4]-delta_angle4);
        ddqr[5] = (1/m[5])*dis[5] - (d[5]/m[5])*(dqr[5]-dot_delta_angle5) - (k[5]/m[5])*(qr[5]-delta_angle5);
        ddqr[6] = (1/m[6])*dis[6] - (d[6]/m[6])*(dqr[6]-dot_delta_angle6) - (k[6]/m[6])*(qr[6]-delta_angle6);

        dqr[i] = dqr[i] + ddqr[i] * 0.001;
        qr[i] = qr[i] + dqr[i] * 0.001;
      }
      //保存数据
      q_file << q << endl;
      dq_file << dq_m << endl;
      qr_file << qr << endl;
      z1_file << z1 << endl;
      z2_file << z2 << endl;
      z3_file << z3 << endl;

      
      //计算误差
      // ''' 位置误差'''
      std::array<double, 7> error_jp{}, error_jv{};
      error_jp[0] = q[0] - qr[0];
      error_jp[1] = q[1] - qr[1] - M_PI/6;
      error_jp[2] = q[2] - qr[2];
      error_jp[3] = q[3] - qr[3] - M_PI/3;
      error_jp[4] = q[4] - qr[4];
      error_jp[5] = q[5] - qr[5] - M_PI/2;
      error_jp[6] = q[6] - qr[6];
      // cout << error_jp << endl;
      // '''速度误差'''
      error_jv[0] = dq_m[0] - dqr[0];
      error_jv[1] = dq_m[1] - dqr[1];
      error_jv[2] = dq_m[2] - dqr[2];
      error_jv[3] = dq_m[3] - dqr[3];
      error_jv[4] = dq_m[4] - dqr[4];
      error_jv[5] = dq_m[5] - dqr[5];
      error_jv[6] = dq_m[6] - dqr[6];
      // 获取各项力
      std::array<double, 7> ine = model.getTorque(q, dq_m, ddq_c, TorqueType::inertia);
      std::array<double, 7> cor = model.getTorque(q, dq_m, ddq_c, TorqueType::coriolis);
      std::array<double, 7> gra = model.getTorque(q, dq_m, ddq_c, TorqueType::gravity);


      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> error_jp_mat(error_jp.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> error_jv_mat(error_jv.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dis_mat(dis.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> cor_mat(cor.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gra_mat(gra.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> ine_mat(ine.data());

      // compute control
      Eigen::VectorXd tau_d(7);
      tau_d = stiffness * error_jp_mat + damping * error_jv_mat + dis_mat;
      // cout << tau_d.transpose() << endl;

      // ESO
      //计算M及其伪逆
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> ddq_c_mat(ddq_c.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_m_mat(dq_m.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> z1_mat(z1.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> z2_mat(z2.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> z3_mat(z3.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dz1_mat(dz1.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dz2_mat(dz2.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dz3_mat(dz3.data());

      Eigen::Matrix<double, 7, 7> M, PPP, ddd;
      Eigen::MatrixXd ddq_pinv = ddq_c_mat.completeOrthogonalDecomposition().pseudoInverse();
      M = ine_mat * ddq_pinv;
      Eigen::Matrix<double, 7, 7> M_inv = M.completeOrthogonalDecomposition().pseudoInverse();

      cout << "M is:\n" << M << endl;
      cout << "M_inv is:\n" << M_inv << endl;

      // std::array<double, 7> eta1, eta2, eta3;
      // for(int i=0;i<7;i++)
      // {
      //   eta1[i] = 800;
      //   eta2[i] = 80;
      //   eta3[i] = 80;
      // }
      double eta1, eta2, eta3, eta;
      eta = 0.5;
      eta1 = 3 * eta;
      eta2 = 3 * eta * eta;
      eta3 = eta * eta * eta;
      Eigen::Matrix<double, 7, 1> mid;
      mid = M_inv * (tau_d - cor_mat - gra_mat - z3_mat);
      for(int i=0;i<7;i++)
      {
        dz1[i] = z2[i] + eta1 * (q[i] - z1[i]);
        dz2[i] = mid(i) + eta2 * (q[i] - z1[i]);
        dz3[i] = eta3 * (q[i] - z1[i]);

        // dz2[0] = mid(0) + 100 * (q[0] - z1[0]);
        // dz2[1] = mid(1) + 100 * (q[1] - z1[1]);
        // dz2[2] = mid(2) + 500 * (q[2] - z1[2]);
        // dz2[3] = mid(3) + 100 * (q[3] - z1[3]);
        // dz2[4] = mid(4) + 500 * (q[4] - z1[4]);
        // dz2[5] = mid(5) + 200 * (q[5] - z1[5]);
        // dz2[6] = mid(6) + 100 * (q[6] - z1[6]);

        // dz3[0] = eta3 * (q[0] - z1[0]);
        // dz3[1] = eta3 * (q[1] - z1[1]);
        // dz3[2] = eta3 * (q[2] - z1[2]);
        // dz3[3] = eta3 * (q[3] - z1[3]);
        // dz3[4] = eta3 * (q[4] - z1[4]);
        // dz3[5] = eta3 * (q[5] - z1[5]);
        // dz3[6] = eta3 * (q[6] - z1[6]);

      }
      for(int i=0;i<7;i++)
      {

        z1[i] = z1[i] + dz1[i] * 0.001;
        z2[i] = z2[i] + dz2[i] * 0.001;
        z3[i] = z3[i] + dz3[i] * 0.001;
      }
      cout << "z3 is: " << z3 << endl;


      Torque cmd(7);
      Eigen::VectorXd::Map(cmd.tau.data(), 7) = tau_d;


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

    q_file.close();
    dq_file.close();
    qr_file.close();
    z1_file.close();
    z2_file.close();
    z3_file.close();


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
