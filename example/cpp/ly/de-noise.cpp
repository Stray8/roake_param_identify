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
#include <Eigen/Dense>
#include <Eigen/SparseLU>

#include <python3.8/Python.h>
#include </home/robot/.local/lib/python3.8/site-packages/numpy/core/include/numpy/arrayobject.h>

using namespace std;
using namespace rokae;
using namespace RtSupportedFields;


int main()
{
    printf("running at line of %d\n",__LINE__);
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
    //读取position
    std::ifstream file_position("/home/robot/robot/roake_param_identify/build/drag/drag_data/position.txt");
    vector<vector<double>> collect_position(22154, vector<double>(7));
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
    std::ifstream file_velocity("/home/robot/robot/roake_param_identify/build/drag/drag_data/velocity.txt");
    vector<vector<double>> collect_velocity(22154, vector<double>(7));
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
    printf("running at line of %d\n",__LINE__);
    Eigen::MatrixXd D_first_order, D_second_order;
    Eigen::MatrixXd q(22154, 7), dq(22154, 7), ddq(22154, 7), hat_q(22154, 7), hat_dq(22153, 7), hat_ddq(22152, 7);
    double gap = 0.001;
    double delta_first_order = 10;
    double delta_second_order = 100;
    for(int i=0;i<22154;i++)
    {
        for(int j=0;j<7;j++)
        {
            q(i,j) = collect_position[i][j];
            dq(i,j) = collect_velocity[i][j];
        }
    }

    printf("running at line of %d\n",__LINE__);
    int N = 22514;
    printf("running at line of %d\n",__LINE__);
    Eigen::SparseMatrix<double> D1(N-1, N);
    Eigen::SparseMatrix<double> D2(N-1, N);
    for(int i=0;i<N-1;i++)
    {
        D1.insert(i, i) = -1;
        D1.insert(i, N-1) = 0;
        D2.insert(i, 0) = 0;
        D2.insert(i, i+1) = 0;
    }
    printf("running at line of %d\n",__LINE__);
    D_first_order = D1 + D2;
    printf("running at line of %d\n",__LINE__);
    Eigen::SparseMatrix<double> D3(N-2, N);
    Eigen::SparseMatrix<double> D4(N-2, N);
    Eigen::SparseMatrix<double> D5(N-2, N);
    for(int i=0;i<N-2;i++)
    {
        D3.insert(i, i) = 1;
        D3.insert(i, N-2) = 0;
        D3.insert(i, N-1) = 0;
        D4.insert(i, 0) = 0;
        D4.insert(i, i+1) = -2;
        D4.insert(i, N-1) = 0;
        D5.insert(i, 0) = 0;
        D5.insert(i, 1) = 0;
        D5.insert(i, i+2) = 0;
    }
    printf("running at line of %d\n",__LINE__);
    D_second_order = D3 + D4 + D5;
    printf("running at line of %d\n",__LINE__);
    // D3 = Eigen::MatrixXd::Identity(N-2, N-2);
    // printf("running at line of %d\n",__LINE__);
    // D4 = -2 * Eigen::MatrixXd::Identity(N-2, N-2);
    // printf("running at line of %d\n",__LINE__);
    // D5 = Eigen::MatrixXd::Zero(N-1, 2);
    // printf("running at line of %d\n",__LINE__);
    // D_second_order = D3 + D4 + D5;
    // printf("running at line of %d\n",__LINE__);
    Eigen::SparseMatrix<double> I(N, N);
    printf("running at line of %d\n",__LINE__);
    I.setIdentity();
    printf("running at line of %d\n",__LINE__);

    Eigen::SparseMatrix<double> A = I + delta_first_order * D_first_order.transpose() * D_first_order + 
                        delta_second_order * D_second_order.transpose() * D_second_order;
    printf("running at line of %d\n",__LINE__);
    // printf("running at line of %d\n",__LINE__);

}