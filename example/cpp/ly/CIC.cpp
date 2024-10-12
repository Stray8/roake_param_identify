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

int main()
{
    ofstream qd0_file;
    ofstream qd1_file;
    ofstream qd2_file;
    ofstream qd3_file;
    ofstream qd4_file;
    ofstream qd5_file;
    ofstream qd6_file;
    qd0_file.open("./ly_data/CIC/qd0.txt");
    qd1_file.open("./ly_data/CIC/qd1.txt");
    qd2_file.open("./ly_data/CIC/qd2.txt");
    qd3_file.open("./ly_data/CIC/qd3.txt");
    qd4_file.open("./ly_data/CIC/qd4.txt");
    qd5_file.open("./ly_data/CIC/qd5.txt");
    qd6_file.open("./ly_data/CIC/qd6.txt");

    ofstream qr0_file;
    ofstream qr1_file;
    ofstream qr2_file;
    ofstream qr3_file;
    ofstream qr4_file;
    ofstream qr5_file;
    ofstream qr6_file;
    qr0_file.open("./ly_data/CIC/qr0.txt");
    qr1_file.open("./ly_data/CIC/qr1.txt");
    qr2_file.open("./ly_data/CIC/qr2.txt");
    qr3_file.open("./ly_data/CIC/qr3.txt");
    qr4_file.open("./ly_data/CIC/qr4.txt");
    qr5_file.open("./ly_data/CIC/qr5.txt");
    qr6_file.open("./ly_data/CIC/qr6.txt");

    static double time=0;
    std::array<double, 10001> qd0, qd1, qd2, qd3, qd4, qd5, qd6;
    std::array<double, 10001> dqd0, dqd1, dqd2, dqd3, dqd4, dqd5, dqd6;
    std::array<double, 10001> ddqd0, ddqd1, ddqd2, ddqd3, ddqd4, ddqd5, ddqd6;
    std::array<double, 10001> qr0, qr1, qr2, qr3, qr4, qr5, qr6;
    std::array<double, 10001> dqr0, dqr1, dqr2, dqr3, dqr4, dqr5, dqr6;
    std::array<double, 10001> ddqr0, ddqr1, ddqr2, ddqr3, ddqr4, ddqr5, ddqr6;
    std::array<double, 10001> f0, f1, f2, f3, f4, f5, f6;
    std::array<double, 10001> e0, e1, e2, e3, e4, e5, e6;
    std::array<double, 10001> de0, de1, de2, de3, de4, de5, de6;

    for(int i=0;i<10001;i++)
    {
        if(i>1500)
        {
            f0[i] = 0;
            f1[i] = 3;
            f2[i] = 6;
            f3[i] = 9;
            f4[i] = 12;
            f5[i] = 15;
            f6[i] = 18;
        }
        qd0[i] = M_PI / 50.0 * (1 - std::cos(M_PI / 4 * 1 * time));
        qd1[i] = M_PI / 50.0 * (1 - std::cos(M_PI / 4 * 3 * time));
        qd2[i] = M_PI / 50.0 * (1 - std::cos(M_PI / 2 * 1 * time));
        qd3[i] = M_PI / 70.0 * (1 - std::cos(M_PI * time));
        qd4[i] = M_PI / 60.0 * (1 - std::cos(M_PI * time));
        qd5[i] = M_PI / 60.0 * (1 - std::cos(M_PI / 2 * 4 * time));
        qd6[i] = M_PI / 70.0 * (1 - std::cos(M_PI / 2 * 3 * time));
        dqd0[i] = 0;
        dqd0[i] = M_PI / 50.0 * M_PI / 4 * 1 * std::sin(M_PI / 4 * 1 * time);
        dqd1[i] = M_PI / 50.0 * M_PI / 4 * 3 * std::sin(M_PI / 4 * 3 * time);
        dqd2[i] = M_PI / 50.0 * M_PI / 2 * 1 * std::sin(M_PI / 2 * 1 * time);
        dqd3[i] = M_PI / 70.0 * M_PI * std::sin(M_PI * time);
        dqd4[i] = M_PI / 60.0 * M_PI * std::sin(M_PI * time);
        dqd5[i] = M_PI / 60.0 * M_PI / 2 * 4 * std::sin(M_PI / 2 * 4 * time);
        dqd6[i] = M_PI / 70.0 * M_PI / 2 * 3 * std::sin(M_PI / 2 * 3 * time);

        if(time>8)
        {
            qd0[i] = 0;
            qd1[i] = 0;
            qd2[i] = 0;
            qd3[i] = 0;
            qd4[i] = 0;
            qd5[i] = 0;
            qd6[i] = 0;

            dqd0[i] = 0;
            dqd1[i] = 0;
            dqd2[i] = 0;
            dqd3[i] = 0;
            dqd4[i] = 0;
            dqd5[i] = 0;
            dqd6[i] = 0;
        }
        // 保存数据
        qd0_file << qd0[i] << endl;
        qd1_file << qd1[i] << endl;
        qd2_file << qd2[i] << endl;
        qd3_file << qd3[i] << endl;
        qd4_file << qd4[i] << endl;
        qd5_file << qd5[i] << endl;
        qd6_file << qd6[i] << endl;

        double m, d, k;
        m = 1;
        d = 10;
        k = 1000;

        for(int i=0;i<10001;i++)
        {
            e0[i] = qr0[i] - qd0[i];
            e1[i] = qr1[i] - qd1[i];
            e2[i] = qr2[i] - qd2[i];
            e3[i] = qr3[i] - qd3[i];
            e4[i] = qr4[i] - qd4[i];
            e5[i] = qr5[i] - qd5[i];
            e6[i] = qr6[i] - qd6[i];

            de0[i] = dqr0[i] - dqd0[i];
            de1[i] = dqr1[i] - dqd1[i];
            de2[i] = dqr2[i] - dqd2[i];
            de3[i] = dqr3[i] - dqd3[i];
            de4[i] = dqr4[i] - dqd4[i];
            de5[i] = dqr5[i] - dqd5[i];
            de6[i] = dqr6[i] - dqd6[i];

            ddqr0[i+1] = (1/m)*f0[i] - (d/m)*de0[i] - (k/m)*e0[i];
            ddqr1[i+1] = (1/m)*f1[i] - (d/m)*de1[i] - (k/m)*e1[i];
            ddqr2[i+1] = (1/m)*f2[i] - (d/m)*de2[i] - (k/m)*e2[i];
            ddqr3[i+1] = (1/m)*f3[i] - (d/m)*de3[i] - (k/m)*e3[i];
            ddqr4[i+1] = (1/m)*f4[i] - (d/m)*de4[i] - (k/m)*e4[i];
            ddqr5[i+1] = (1/m)*f5[i] - (d/m)*de5[i] - (k/m)*e5[i];
            ddqr6[i+1] = (1/m)*f6[i] - (d/m)*de6[i] - (k/m)*e6[i];

            dqr0[i+1] = dqr0[i] + ddqr0[i+1] * 0.001;
            dqr1[i+1] = dqr1[i] + ddqr1[i+1] * 0.001;
            dqr2[i+1] = dqr2[i] + ddqr2[i+1] * 0.001;
            dqr3[i+1] = dqr3[i] + ddqr3[i+1] * 0.001;
            dqr4[i+1] = dqr4[i] + ddqr4[i+1] * 0.001;
            dqr5[i+1] = dqr5[i] + ddqr5[i+1] * 0.001;
            dqr6[i+1] = dqr6[i] + ddqr6[i+1] * 0.001;

            qr0[i+1] = qr0[i] + dqr0[i+1] * 0.001;
            qr1[i+1] = qr1[i] + dqr1[i+1] * 0.001;
            qr2[i+1] = qr2[i] + dqr2[i+1] * 0.001;
            qr3[i+1] = qr3[i] + dqr3[i+1] * 0.001;
            qr4[i+1] = qr4[i] + dqr4[i+1] * 0.001;
            qr5[i+1] = qr5[i] + dqr5[i+1] * 0.001;
            qr6[i+1] = qr6[i] + dqr6[i+1] * 0.001;
        }
        // 保存数据
        qr0_file << qr0[i] << endl;
        qr1_file << qr1[i] << endl;
        qr2_file << qr2[i] << endl;
        qr3_file << qr3[i] << endl;
        qr4_file << qr4[i] << endl;
        qr5_file << qr5[i] << endl;
        qr6_file << qr6[i] << endl;
        time += 0.001;
    }

    qd0_file.close();
    qd1_file.close();
    qd2_file.close();
    qd3_file.close();
    qd4_file.close();
    qd5_file.close();
    qd6_file.close();

    qr0_file.close();
    qr1_file.close();
    qr2_file.close();
    qr3_file.close();
    qr4_file.close();
    qr5_file.close();
    qr6_file.close();
}