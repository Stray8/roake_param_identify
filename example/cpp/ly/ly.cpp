#include <python3.8/Python.h>
#include </home/robot/.local/lib/python3.8/site-packages/numpy/core/include/numpy/arrayobject.h>
#include <iostream>
#include <cmath>
#include <thread>
#include <fstream>
#include <vector>
// #include <Eigen/Eigen>

int main()
{
    printf("running at line of %d\n",__LINE__);
    //初始化接口
    Py_Initialize();
    if(!Py_IsInitialized())
    {
        std::cout << "python init failed" << std::endl;
        return 1;
    }
    import_array();
    printf("running at line of %d\n",__LINE__);
    //初始化python系统文件路径，保证可以访问到.py文件
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/robot/robot/roake_param_identify/example/cpp/ly')");
    printf("running at line of %d\n",__LINE__);
    //调用python文件名，不用写后缀
    PyObject* trainingSet = PyImport_ImportModule("GP_fitting1");
    if(trainingSet == nullptr)
    {
        std::cout << "module not found: trainingSet" << std::endl;
        return 1;
    }
    printf("running at line of %d\n",__LINE__);

    PyObject* args = PyTuple_New(1); //2是指传进来2个参数
    //获取函数
    PyObject* prediction = PyObject_GetAttrString(trainingSet, "prediction");
    npy_intp dims[2] = {1, 21};

    if(!prediction)
    {
        std::cout << "class not found: prediction" << std::endl;
        return 1;
    }
    printf("running at line of %d\n",__LINE__);
    //给函数传递参数
    // double X[1][21] = {{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21}};

    for(int i=0; i<10; i++)
    {
        double X[1][21] = {{2.91451863e-05, 5.23692755e-01, 2.16535807e-05, 1.04732677e+00,
                            8.65660769e-04, 1.57096273e+00, 6.44935560e-04, 4.97621790e-05,
                            6.48620157e-03, 1.83847530e-03, 8.42785672e-03 ,4.77093191e-02,
                            1.06984107e-02, 2.77792651e-02, 1.80193148e-03, 1.19337800e-01,
                            3.74046877e-02, 1.50298651e-01, 7.46985098e-01 ,1.78475181e-01,
                            3.94134328e-01}};
        PyObject* pintput_X = PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, (void*)X);
        PyTuple_SetItem(args, 0, pintput_X);
        PyObject* pRet = PyObject_CallObject(prediction, args);
        PyArrayObject* pArray = (PyArrayObject*)(pRet);
        double* result = (double*)(PyArray_DATA(pArray));
        for (int i = 0; i < 7; i++)
        {
            printf("%.8f  ", result[i]); 
        }
        printf("\n");
    }
    // Py_Finalize();
    return 0;
}
