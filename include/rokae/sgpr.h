#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <cmath>
#include <fstream>
// #include <boost/math/distributions/normal.hpp>
#include <ceres/solver.h>

using namespace Eigen;
using namespace std;

class sgpr
{
private:
    MatrixXd X;
    VectorXd y;
    double likelihood_noise;
    int restart;
    VectorXd param;
    MatrixXd cov_y_y;
    VectorXd beta;
    MatrixXd inv_cov_y_y;
public:
    // sgpr(MatrixXd X, VectorXd y, int restart = 1) : X(X), y(y), restart(restart)
    sgpr(MatrixXd X, VectorXd y, double likelihood_noise = 0.1, int restart = 1) : X(X), y(y), likelihood_noise(likelihood_noise), restart(restart)
    {
        int input_dim = X.cols();
        int input_num = X.rows();
        int q_dim = input_dim / 3;
        std::vector<double> param(q_dim * 4 + 2);
    } 
    void init_random_param()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> normal_list(0.0, 1.0);
        VectorXd sqrt_kls_m = 0.01 * (normal_list(gen) * VectorXd::Ones(X.cols() / 3 * 2) + 2 * VectorXd::Ones(X.cols() / 3 * 2));
        VectorXd sqrt_kls_c = 0.01 * (normal_list(gen) * VectorXd::Ones(X.cols() / 3 * 2) + 2 * VectorXd::Ones(X.cols() / 3 * 2));
        double kn_m = 0.1 * normal_list(gen);
        double kn_c = 0.1 * normal_list(gen);
        param << kn_m, sqrt_kls_m, kn_c, sqrt_kls_c;
    }
    //rbf_single
    Eigen::MatrixXd rbf_single(const Eigen::MatrixXd& x, const Eigen::MatrixXd& x_, const Eigen::VectorXd& param)
    {
        double kn = param[0];
        Eigen::VectorXd sqrt_kls = param.tail(param.size() - 1);
        std::vector<std::vector<double>> diffs(x.size(), std::vector<double>(x_[0].size(), 0.0))
        for(int i = 0; i < x.size(); ++i)
        {
            for(int j = 0; j < x_[0].size(); ++j)
            {
                diffs[i][j] = (x[i][j] / sqrt_kls[i] - (x_[i][j] / sqrt_kls[j]));
            }
        }
        // Eigen::VectorXd x_scaled = x.array().rowwise() / sqrt_kls().transpose().array();
    }
//     Eigen::MatrixXd expanded_x = x.replicate(1, x_.rows());
//     Eigen::MatrixXd x_div_sqrt_kls = expanded_x.array().colwise() / sqrt_kls.array();

//     Eigen::MatrixXd x_scaled = (x.array().colwise() / sqrt_kls()).matrix();
//     Eigen::MatrixXd x__scaled = (x_.array().colwise() / sqrt_kls()).matrix();
//     Eigen::MatrixXd diffs(x.rows(), x_rows());
//     for(int i = 0; i < x.rows(); ++i)
//     {
//         for(int j = 0; j < x_.rows(); ++j)
//         {
//             diffs(i, j) = (x_scaled.row(i) - x__scaled.row(j)).squaredNorm();
//         }
//     }
//     Eigen::VectorXd norm_x = x.rightCols(x.cols() - X.cols() / 3).rowwise().norm();
//     Eigen::VectorXd norm_x_ = x.rightCols(x_.cols() - X.cols() / 3).rowwise().norm();
//     Eigen::MatrixXd temp1 = (0.1 * norm_x.array()).tanh().matrix().asDiagonal();
//     Eigen::MatrixXd temp2 = (0.1 * norm_x_.array()).tanh().matrix().asDiagonal();
//     Eigen::MatrixXd kernel = (-0.5 * diffs.array()).exp().matrix();
//     kernel = kn * kn * kernel;
//     return kernel;
//    }

    //Multivariate normal log PDF
    double mvn_logpdf(const Eigen::VectorXd& y, const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov)
    {
        Eigen::LLT<Eigen::MatrixXd> lltOfCov(cov);
        Eigen::MatrixXd L = lltOfCov.matrixL();
        double logDet = 2.0 * L.diagonal().array().log().sum();
        Eigen::VectorXd diff = y - mean;
        double quadForm = (L.triangularView<Eigen::Lower>().solve(diff).squaredNorm());
        double logProb = -0.5 * (logDet + quadForm + X.rows() * std::log(2.0 * M_PI));
        return logProb;
    }

    // double build_Objective(const Eigen::VectorXd& param)
    // {
    //     cov_y_y = rbf(X, X, param) + likelihood_noise * likelihood_noise * MatrixXd::Identity(X.rows(), X.rows());
    //     double out = -mvn_logpdf(y, Eigen::VectorXd::Zero(X.rows()), cov_y_y);
    //     return out;
    // }

    // Eigen::MatrixXd predict_determined_input(MatrixXd input)
    // {
    //     Eigen::MatrixXd cov_y_f = rbf(X, input, param);
    //     Eigen::MatrixXd mean_outputs = cov_y_f.transpose() * beta;
    //     return mean_outputs;
    // }




    // void set_param(VectorXd new_param)
    // {
    //     param = new_param;
    //     cov_y_y = rbf(X, X, param) + likelihood_noise * likelihood_noise * MatrixXd::Identity(X.rows(), X.rows());
    //     beta = cov_y_y.ldlt().solve(y);
    //     inv_cov_y_y = cov_y_y.ldlt().solve(MatrixXd::Identity(X.rows(), X.rows()));
    // }



    // void train()
    // {
    //     double max_logpdf = -1e20;
    //     for(int i = 0; i < restart; i++)
    //     {
            

    //     }

    // }

    // void build_Objective(const Eigen::VectorXd& param)
    // {
    //     cov_y_y = rbf(X, X, param) + likelihood_noise * likelihood_noise * MatrixXd::Identity(X.rows(), X.rows());
    //     double out = -mvn_logpdf(y, Eigen::VectorXd::Zero(X.rows()), cov_y_y);
    //     return out;
    // }
};
    

    
    // void set_param(VectorXd param_)
    // {
    //     param = param_;
    //     cov_y_y = rbf(X, X, param) + likelihood_noise * likelihood_noise * MatrixXd::Identity(input_num, input_num);
    //     beta = cov_y_y.ldlt().solve(y);
    //     inv_cov_y_y = cov_y_y.ldlt().solve(MatrixXd::Identity(input_num, input_num));
    // }

    // void build_objective(VectorXd param_)
    // {
    //     cov_y_y = rbf(X, X, param_) + likelihood_noise * likelihood_noise * MatrixXd::Identity(input_num, input_num);
    //     out = -mutivariate_normal_logpdf(y, cov_y_y);
    //     return out;
    // }
    // ~sgpr();








// using namespace Eigen;
// using namespace std;

// class SGPR {
// private:
//     MatrixXd X;
//     VectorXd y;
//     double likelihood_noise;
//     VectorXd param;
//     MatrixXd cov_y_y;
//     VectorXd beta;

//     // 生成正态分布的随机数
//     double normal_random(double mean = 0.0, double stddev = 1.0) {
//         static random_device rd;
//         static mt19937 gen(rd());
//         normal_distribution<> d(mean, stddev);
//         return d(gen);
//     }

//     // 初始化参数
//     void init_random_param() {
//         for (int i = 0; i < param.size(); ++i) {
//             param(i) = normal_random(2, 0.01);
//         }
//     }

//     // RBF核函数
//     MatrixXd rbf(const MatrixXd& x1, const MatrixXd& x2, const VectorXd& params) {
//         double kn = params(0);
//         VectorXd sqrt_kls = params.segment(1, x1.cols());
//         MatrixXd result(x1.rows(), x2.rows());
//         for (int i = 0; i < x1.rows(); ++i) {
//             for (int j = 0; j < x2.rows(); ++j) {
//                 VectorXd diffs = (x1.row(i).cwiseQuotient(sqrt_kls) - x2.row(j).cwiseQuotient(sqrt_kls)).array().square();
//                 result(i, j) = kn * std::exp(-0.5 * diffs.sum());
//             }
//         }
//         return result;
//     }

// public:
//     SGPR(const MatrixXd& X, const VectorXd& y, double likelihood_noise = 0.1)
//         : X(X), y(y), likelihood_noise(likelihood_noise) {
//         int input_dim = X.cols();
//         int q_dim = input_dim / 3;
//         param.resize(q_dim * 4 + 2);
//         init_random_param();
//     }

//     void train() {
//         cov_y_y = rbf(X, X, param) + likelihood_noise * MatrixXd::Identity(X.rows(), X.rows());
//         beta = cov_y_y.ldlt().solve(y);
//     }

//     VectorXd predict(const MatrixXd& inputs) {
//         MatrixXd cov_y_f = rbf(X, inputs, param);
//         VectorXd mean_outputs = cov_y_f.transpose() * beta;
//         return mean_outputs;
//     }
// };