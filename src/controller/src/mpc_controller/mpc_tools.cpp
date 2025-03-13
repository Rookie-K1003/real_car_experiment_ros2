/*
文件名: MPC工具
作者: kq
完成时间: 2025.03.13

编译类型: 动态库

依赖: 
    外部库:
      osqp
*/

#include "mpc_tools.h"

namespace Controller
{
    template
    class SparseMatrix<c_float>;

    template
    class QPProblem<c_float>;

    template<typename T>
    SparseMatrix<T>::SparseMatrix() {
        m_ = n_ = 0;
    }

    template<typename T>
    SparseMatrix<T>::~SparseMatrix() {
        elements_.clear();
        freeOSQPCSCInstance();
    }

    template<typename T>
    void SparseMatrix<T>::freeOSQPCSCInstance() {
        osqp_csc_data_.clear();
        osqp_csc_row_idx_.clear();
        osqp_csc_col_start_.clear();

        if(osqp_csc_instance != nullptr) {
            c_free(osqp_csc_instance);
            osqp_csc_instance = nullptr;
            /*
                释放内存后，将osqp_csc_instance指针设置为nullptr，
                这是一个良好的编程习惯，可以防止野指针的出现
            */
        }
    }

    template<typename T>
    void SparseMatrix<T>::initialize(int m, int n) {
        m_ = m;
        n_ = n;
        elements_.clear();
    }

    template<typename T>
    void SparseMatrix<T>::addElement(int r, int c, T v) {
        elements_.push_back({r, c, v});
    }

    template<typename T>
    csc* SparseMatrix<T>::toOSQPCSC() {
        freeOSQPCSCInstance();

        sort(elements_.begin(), elements_.end());

        int idx = 0;
        int n_elem = elements_.size();

        osqp_csc_col_start_.push_back(0);
        for(int c = 0; c < n_; c++) {
            while((idx < n_elem) && elements_[idx].c == c) {
                osqp_csc_data_.push_back(elements_[idx].v);
                osqp_csc_row_idx_.push_back(elements_[idx].r);
                idx++;
            }

            osqp_csc_col_start_.push_back(osqp_csc_data_.size());
        }

        osqp_csc_instance = csc_matrix(m_, n_, osqp_csc_data_.size(), osqp_csc_data_.data(), osqp_csc_row_idx_.data(), osqp_csc_col_start_.data());
        return osqp_csc_instance;
    }

    template<typename T>
    QPProblem<T>::~QPProblem() {
        if(osqp_workspace_ != nullptr) {
            osqp_workspace_->data->P = nullptr;
            osqp_workspace_->data->q = nullptr;

            osqp_workspace_->data->A = nullptr;
            osqp_workspace_->data->l = nullptr;
            osqp_workspace_->data->u = nullptr;

            //cleanup workspace
            osqp_cleanup(osqp_workspace_);
        }
    }

    template<typename T>
    void QPProblem<T>::initialize(int n, int m) {
        n_ = n;
        m_ = m;

        A_.initialize(m_, n_);
        l_.resize(m_);
        u_.resize(m_);

        std::fill(l_.begin(), l_.end(), 0);
        std::fill(u_.begin(), u_.end(), 0);

        P_.initialize(n_, n_);
        q_.resize(n_);

        std::fill(q_.begin(), q_.end(), 0);
    }

    template<typename T>
    /**
     * @brief 解决一个二次规划问题
     *
     * 该函数使用OSQP库来解决二次规划问题，并返回求解结果。
     *
     * @param error_code 用于存储求解过程中可能出现的错误代码的指针
     *
     * @return 指向OSQPSolution结构的指针，该结构包含求解结果
     */
    OSQPSolution* QPProblem<T>::solve(int *error_code) {
        //set up workspace
        if(osqp_workspace_ == nullptr) {
            osqp_settings_ = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
            osqp_data_     = (OSQPData *)    c_malloc(sizeof(OSQPData));

            //populate data
            osqp_data_->n = n_;
            osqp_data_->m = m_;

            osqp_data_->A = A_.toOSQPCSC();
            osqp_data_->l = l_.data();
            osqp_data_->u = u_.data();

            osqp_data_->P = P_.toOSQPCSC();
            osqp_data_->q = q_.data();

            osqp_set_default_settings(osqp_settings_);
            osqp_setup(&osqp_workspace_, osqp_data_, osqp_settings_);
        }
        else {
            csc *A_csc = A_.toOSQPCSC();
            osqp_update_A(osqp_workspace_, A_csc->x, NULL, A_csc->nzmax);
            osqp_update_bounds(osqp_workspace_, l_.data(), u_.data());

            csc *P_csc = P_.toOSQPCSC();
            osqp_update_P(osqp_workspace_, P_csc->x, NULL, P_csc->nzmax);
            osqp_update_lin_cost(osqp_workspace_, q_.data());
        }

        *error_code = osqp_solve(osqp_workspace_);

        return osqp_workspace_->solution;
    }


} // namespace Controller