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

int main()
{
    ofstream qd0_file;
    qd0_file.open("./ly_data/CIC/qd0.txt");
    ofstream qr0_file;
    qr0_file.open("./ly_data/CIC/qr0.txt");

    static double time=0;
    std::array<double, 10001> qd0;
    std::array<double, 10001> dqd0;
    std::array<double, 10001> ddqd0;
    std::array<double, 10001> qr0;
    std::array<double, 10001> dqr0;
    std::array<double, 10001> ddqr0;
    std::array<double, 10001> f0;
    std::array<double, 10001> e0;
    std::array<double, 10001> de0;

    for(int i=0;i<10001;i++)
    {
        if(i>1500)
        {
            f0[i] = 100;
        }
        qd0[i] = 2;
        dqd0[i] = 0;
        time += 0.001;
        // 保存数据
        qd0_file << qd0[i] << endl;
    }
    double m, d, k;
    m = 1;
    d = 10;
    k = 100;
    for(int i=0;i<10001;i++)
    {
        e0[i] = qr0[i] - qd0[i];
        de0[i] = dqr0[i] - dqd0[i];

        ddqr0[i+1] = (1/m)*f0[i] - (d/m)*de0[i] - (k/m)*e0[i];
        dqr0[i+1] = dqr0[i] + ddqr0[i+1] * 0.001;
        qr0[i+1] = qr0[i] + dqr0[i+1] * 0.001;

        // 保存数据
        qr0_file << qr0[i] << endl;
    }

    qd0_file.close();
    qr0_file.close();
}