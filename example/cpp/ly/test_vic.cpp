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
    printf("running at line of %d\n",__LINE__);
    //初始化接口
    Py_Initialize();
    import_array();
    //初始化python系统文件路径，保证可以访问到.py文件
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/robot/robot/roake_param_identify/example/cpp/ly/simulation/algorithm')");
    printf("running at line of %d\n",__LINE__);
    //调用python文件名，不用写后缀
    printf("running at line of %d\n",__LINE__);
    PyObject* OneStep_Impedance_Cons = PyImport_ImportModule("OneStep_Impedance_Cons");
    PyObject* args = PyTuple_New(1); //2是指传进来2个参数
    printf("running at line of %d\n",__LINE__);
    //获取函数
    PyObject* out = PyObject_GetAttrString(OneStep_Impedance_Cons, "out");
    npy_intp dims[2] = {2, 7};
    printf("running at line of %d\n",__LINE__);
    std::array<double, 7> x, dx;
    for(int i=0;i<7;i++)
    {
        x[i] = i;
        dx[i] = 0.01 * i;
    }
    double input[2][7];
    for(int i=0;i<7;i++)
    {
        input[0][i] = x[i];
        input[1][i] = dx[i];
    }
    cout << "input is\n" << input[0][1] << endl;
    PyObject* pintput_X = PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, (void*)input);
    PyTuple_SetItem(args, 0, pintput_X);
    PyObject* pRet = PyObject_CallObject(out, args);
    printf("running at line of %d\n",__LINE__);
    PyArrayObject* pArray = (PyArrayObject*)(pRet);
    double* result = (double*)(PyArray_DATA(pArray));
    printf("running at line of %d\n",__LINE__);
    std::cout << "result is: " << result << std::endl;

    // for(int i=0; i<7; i++)
    // {
    //     mu[i] = result[i];
    // }
    // std::cout << "mu is: " << mu << std::endl;
    
}
