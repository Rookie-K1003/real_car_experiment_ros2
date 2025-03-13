#ifndef MPC_TOOLS_H__
#define MPC_TOOLS_H__

/*
    等价于原ros1代码的MPC.h，主要是封装了一些MPC的工具
*/
#include "osqp.h"
#include <vector>
#include <cmath>
#include <algorithm>

namespace Controller {
    struct HardConstraint {
        double max_steer;

        double max_accel;
        double min_accel;

        double max_steer_rate;
        double max_jerk;
    };

    struct CostFunctionWeights {
        double w_position;
        double w_angle;

        double w_velocity;

        double w_accel;
        double w_jerk;

        double w_steer;
        double w_dsteer;
    };

    struct State {
        double x;
        double y;
        double yaw;
        double v;

        double steer_angle;
        double accel;
    };

    struct Parameters {
        int pred_horizon;
        int control_horizon;

        double dt;
    };

    struct Model {
        double l_f;
        double l_r;
        double m;
    };

    struct ControlOutput {
        int error_code;
        double steer;
        double accel;
    };

    // 定义一个稀疏矩阵的元素
    template<typename T>
    struct SparseMatrixElement {
        int r, c;
        T v;

        inline bool operator<(const SparseMatrixElement &rhs) {
            return (c == rhs.c) ? (r < rhs.r) : (c < rhs.c);
        }
    };

    // 稀疏矩阵模板类
    template<typename T>
    class SparseMatrix {
    private:
        int m_, n_;

        std::vector< SparseMatrixElement<T> > elements_;

        std::vector<T> osqp_csc_data_;
        std::vector<c_int> osqp_csc_row_idx_;
        std::vector<c_int> osqp_csc_col_start_;
        csc *osqp_csc_instance = nullptr;

        void freeOSQPCSCInstance();

    public:
        SparseMatrix();
        ~SparseMatrix();

        void initialize(int m, int n);
        void addElement(int r, int c, T v); //向稀疏矩阵中添加一个非零元素
        csc *toOSQPCSC(); //返回OSQP的CSC格式
    };

    template<typename T>
    class QPProblem {
    private:
        OSQPWorkspace *osqp_workspace_ = nullptr;
        OSQPSettings  *osqp_settings_= nullptr;
        OSQPData      *osqp_data_ = nullptr;

    public:
        //number of variables and constraints
        int n_, m_;

        //constraints
        SparseMatrix<T> A_;
        std::vector<T> l_, u_;

        //cost function
        SparseMatrix<T> P_; // QP costfunction中的二次项矩阵
        std::vector<T> q_; // QP costfunction中的线性项矩阵

        ~QPProblem();
        void initialize(int n, int m);
        OSQPSolution* solve(int *error_code);
    };

} // namespace Controller

#endif // MPC_TOOLS_H__