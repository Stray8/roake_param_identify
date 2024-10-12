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
    printf("running at line of %d\n",__LINE__);
    //初始化接口
    Py_Initialize();
    import_array();
    //初始化python系统文件路径，保证可以访问到.py文件
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/robot/robot/roake_param_identify/example/cpp/ly')");
    //调用python文件名，不用写后缀
    PyObject* trainingSet = PyImport_ImportModule("GP_SGPR");
    PyObject* args = PyTuple_New(1); //2是指传进来2个参数
    //获取函数
    PyObject* prediction = PyObject_GetAttrString(trainingSet, "prediction");
    npy_intp dims[2] = {1, 21};

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
    stiffness << -2500, 0, 0, 0, 0, 0, 0,
                 0, -2500, 0, 0, 0, 0, 0,
                 0, 0, -2500, 0, 0, 0, 0,
                 0, 0, 0, -2500, 0, 0, 0,
                 0, 0, 0, 0, -1500, 0, 0,
                 0, 0, 0, 0, 0, -1500, 0,
                 0, 0, 0, 0, 0, 0, -1500;
    damping << -50, 0, 0, 0, 0, 0, 0,
               0, -50, 0, 0, 0, 0, 0,
               0, 0, -50, 0, 0, 0, 0,
               0, 0, 0, -50, 0, 0, 0,
               0, 0, 0, 0, -30, 0, 0,
               0, 0, 0, 0, 0, -30, 0,
               0, 0, 0, 0, 0, 0, -30;
 
    std::array<double, 16> init_position {};
    Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), init_position);

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
    std::ifstream file_position("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_collect/hat_p.txt");
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
    std::ifstream file_velocity("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_collect/hat_dp.txt");
    vector<vector<double>> collect_velocity(11076, vector<double>(7));
    rows = 0;
    cols = 0;
    while (getline(file_velocity, line))
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
    // 读取acceleration
    std::ifstream file_acceleration("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_collect/hat_ddp.txt");
    vector<vector<double>> collect_acceleration(11076, vector<double>(7));
    rows = 0;
    cols = 0;
    while (getline(file_acceleration, line))
    {
        vector<string> tokens = split_func(line, ' ');
        cols = 0;
        for (const auto &t : tokens)
        {
            collect_acceleration[rows][cols] = stod(t);
            cols++;
        }
        rows++;
    }

    ofstream position_file;
    ofstream position_error_file;
    ofstream velocity_file;
    ofstream inertia_file;
    ofstream coriolis_file;
    ofstream gravity_file;
    ofstream torque_file;
    position_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s4/position.txt");
    position_error_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s4/position_error.txt");
    velocity_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s4/velocity.txt");
    inertia_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s4/inertia.txt");
    coriolis_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s4/coriolis.txt");
    gravity_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s4/gravity.txt");
    torque_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s4/torque.txt");
    
    // position_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_PD/position.txt");
    // position_error_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_PD/position_error.txt");
    // velocity_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_PD/velocity.txt");
    // inertia_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_PD/inertia.txt");
    // coriolis_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_PD/coriolis.txt");
    // gravity_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_PD/gravity.txt");
    // torque_file.open("/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_PD/torque.txt");

    std:array<double, 7> dq_before{}, ddq_d_before{};
    for(int i=0;i<7;i++)
    {
        dq_before[i] = 0;
        ddq_d_before[i] = 0;
    }
    std::function<Torque(void)> callback = [&]
    {
      static double time = 0;
      time += 0.002;
      //定义参考轨迹
      static int times = 0;
      std::array<double, 7> q_d{}, dq_d{}, ddq_d{};
      // '''参考位置''' collect_position
      // 参考速度 collect_velocity
      // 参考加速度
      for(int i=0;i<7;i++)
      {
        q_d[i] = collect_position[times][i];
        dq_d[i] = collect_velocity[times][i];
        ddq_d[i] = collect_acceleration[times][i];
      }
      // for(int i=0;i<7;i++)
      // {
      //   ddq_d[i] = 0.5 * ddq_d_before[i] + 0.5 * ddq_d[i];
      // }
      // ddq_d_before = ddq_d;
      times += 1;
      // 接收设置为true, 回调函数中可以直接读取
      std::array<double, 7> q{}, dq{}, ddq{}, tau{};
      q = robot.jointPos(ec);
      dq = robot.jointVel(ec);
      for(int i=0;i<7;i++)
      {
        ddq[i] = (dq[i] - dq_before[i]) / 0.001;
      }
      dq_before = dq;
      tau = robot.jointTorque(ec);
      // 获取各项力
      std::array<double, 7> ine = model.getTorque(q, dq, ddq, TorqueType::inertia);
      std::array<double, 7> cor = model.getTorque(q, dq, ddq, TorqueType::coriolis);
      std::array<double, 7> gra = model.getTorque(q, dq, ddq, TorqueType::gravity);
      // 计算mu
      double X[1][21];
      std::array<double, 7> mu{};
      for(int i=0; i<7; i++)
      {
        X[0][i]=q[i];
        X[0][i+7]=dq_d[i];
        X[0][i+14]=ddq_d[i];
      }
      PyObject* pintput_X = PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, (void*)X);
      PyTuple_SetItem(args, 0, pintput_X);
      PyObject* pRet = PyObject_CallObject(prediction, args);
      PyArrayObject* pArray = (PyArrayObject*)(pRet);
      double* result = (double*)(PyArray_DATA(pArray));
      for(int i=0; i<7; i++)
      {
        mu[i] = result[i];
      }
      // cout << mu << endl;
      //计算误差
      // ''' 位置误差'''
      std::array<double, 7> error_jp{}, error_jv{};
      for(int i=0;i<7;i++)
      {
        error_jp[i] = q[i] - q_d[i];
      }
      // cout << error_jp << endl;
      // '''速度误差'''
      for(int i=0;i<7;i++)
      {
        error_jv[i] = dq[i] - dq_d[i];
      }
      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> error_jp_mat(error_jp.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> error_jv_mat(error_jv.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> ine_mat(ine.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> cor_mat(cor.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gra_mat(gra.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> mu_mat(mu.data());
      // compute control
      Eigen::VectorXd tau_ff(7);
      tau_ff = stiffness * error_jp_mat + damping * error_jv_mat;
      Eigen::VectorXd tau_fb(7);
      tau_fb = mu_mat;
      Eigen::VectorXd tau_d(7);
      // tau_d = tau_ff;
      tau_d = tau_ff + tau_fb;
    //   cout << tau_d.transpose() << endl;

      Torque cmd(7);
      Eigen::VectorXd::Map(cmd.tau.data(), 7) = tau_d;
      cout <<cmd.tau<< endl;
      // 保存数据
      position_file << q << endl;
      position_error_file << error_jp << endl;
      velocity_file << dq << endl;
      inertia_file << ine << endl;
      coriolis_file << cor << endl;
      gravity_file << gra << endl;
      torque_file << tau << endl;

      if(time > 30)
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
