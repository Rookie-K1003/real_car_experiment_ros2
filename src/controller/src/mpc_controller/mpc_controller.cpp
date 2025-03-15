/*
文件名: MPC具体函数的实现,完整代码V1.0
作者: kq
完成时间: 2025.03.14

编译类型: 动态库

依赖: 
    ros2相关：
        nav_msgs
        rclcpp
    外部库：
        config_reader
        base_msgs
        mpc_tools
        path_adapter
*/

#include "mpc_controller.h"

namespace Controller
{
    double gVelocity = 2.0; // 全局速度参考速度，暂时设置为常量
    double gMinVelocity = 0.5; // 全局速度最小参考速度，暂时设置为常量
    double gCurvatureK = 150.0;
    

    MPCController::MPCController(): Node("mpc_controller")
    {
        // 读取配置文件
        mpc_config_ = std::make_unique<ConfigReader>();
        mpc_config_->read_mpc_param();
        MpcParamStruct params;
        params = mpc_config_->mpc_param();
        mpc_config_->read_mpc_model();
        MpcModelStruct model;
        model = mpc_config_->mpc_model();
        mpc_config_->read_mpc_hard_constraint();
        MpcHardConstraintStruct hard_constraint;
        hard_constraint = mpc_config_->mpc_hard_constraint();
        mpc_config_->read_mpc_cost_weight();
        MpcCostWeightStruct cost_weight;
        cost_weight = mpc_config_->mpc_cost_weight();

        // 将配置参数赋值给成员变量
        Initialize(params, model, hard_constraint, cost_weight);

        ref_path_sub_ = this->create_subscription<Path>(
            "/planning/global_path_planning", rclcpp::SensorDataQoS(), std::bind(&MPCController::RefPathCallback, this, std::placeholders::_1));

        path_adapter_ = std::make_unique<PathAdapter>();
    }

    /**
     * @brief 运行一次MPC控制器
     *
     * 该函数用于执行MPC控制器的一次运算周期。
     *
     * @return 返回值为布尔类型，表示MPC控制器是否成功运行一次。成功返回true，失败返回false。
     */
    bool MPCController::RunOnce()
    {
        RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "MPC Controller running..."); // 打印测试
        LogTest();

        // // ~ step 1: 获取当前状态、上一次的控制量
        // State state;
        // state = GetStateAndLastControl();

        // // ~ step 2: 时域内的轨迹计算
        // PoseStamped start_pose;
        // std::vector<PoseStamped> track;
        // start_pose = GetStartPose();

        // path_adapter_->getPath(start_pose, param_.dt, gVelocity, gMinVelocity, 
        //                         gCurvatureK, fabs(hard_constraint_.min_accel), 
        //                         param_.pred_horizon - 1, &track);
        // // ~ step 3: MPC解算
        // ControlOutput control_output;
        // std::vector<State> prediction_output;

        // // TODO: 使用参数配置文件调参，目前迭代次数设为3次，迭代门限为0.01
        // IterativeUpdate(state, track, 3, 0.01, &control_output, &prediction_output);

        // // ~ step 4: 输出控制量

        
        return true;
    }

    void MPCController::Initialize(const MpcParamStruct &parameters, const MpcModelStruct &model, const MpcHardConstraintStruct &constraint, const MpcCostWeightStruct &weights)
    {
        // 参数
        param_.dt = parameters.dt_;
        param_.pred_horizon = parameters.pred_horizon_;
        param_.control_horizon = parameters.control_horizon_;

        // 动力学模型
        model_.l_f = model.lf_;
        model_.l_r = model.lr_;
        model_.m = model.m_;

        // 硬约束
        hard_constraint_.max_steer = constraint.max_steer_;
        hard_constraint_.max_accel = constraint.max_acceleration_;
        hard_constraint_.min_accel = constraint.min_acceleration_;
        hard_constraint_.max_steer_rate = constraint.max_steer_rate_;
        hard_constraint_.max_jerk = constraint.max_jerk_;

        // 代价权重
        cost_weight_.w_position = weights.w_position_;
        cost_weight_.w_angle = weights.w_heading_;
        cost_weight_.w_velocity = weights.w_velocity_;
        cost_weight_.w_steer = weights.w_steer_;
        cost_weight_.w_accel = weights.w_acceleration_;
        cost_weight_.w_dsteer = weights.w_steer_rate_;
        cost_weight_.w_jerk = weights.w_jerk_;
    }

    void MPCController::LogTest()
    {
        // // 测试config_reader
        // RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "prediction horizon: %d", param_.pred_horizon);
        // RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "hard_constraint max_steer: %f", hard_constraint_.max_steer);

        // 测试参考轨迹获取
        std::vector<Point> waypoint_test = path_adapter_->waypoints();
        RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "ref_traj_size: %d", waypoint_test.size());
        if (waypoint_test.size() > 0) {
            RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "first point x: %f", waypoint_test[0].x);
            RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "first point y: %f", waypoint_test[0].y);
        }
    }

    /**
     * @brief 规范化角度
     *
     * 将输入的角度值规范化到 [-π, π) 范围内。
     *
     * @param a 输入的角度值（弧度制）
     *
     * @return 规范化后的角度值（弧度制）
     */
    double MPCController::NormalizeAngle(double a)
    {
        return fmod(fmod(a + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI;
    }

    State MPCController::GetStateAndLastControl()
    {
        State state;
        state.x = state_.x;
        state.y = state_.y;
        state.yaw = NormalizeAngle(state_.phi); // 注意角度规范化
        state.v = sqrt(state_.v_x * state_.v_x + state_.v_y * state_.v_y);
        state.steer_angle = last_steer_angle_;
        state.accel = last_accel_;
        
        return state;
    }

    PoseStamped MPCController::GetStartPose()
    {
        PoseStamped pose;
        pose.x = state_.x;
        pose.y = state_.y;
        pose.yaw = NormalizeAngle(state_.phi);
        pose.v = state_.v_x; // 这里暂时只考虑x方向的速度
        pose.t = 0;
        return pose;
    }

    /**
     * @brief 更新模型预测控制器的状态，并计算控制输出和预测结果
     *
     * 使用当前状态和线性化点，以及参考轨迹，更新模型预测控制器的状态，并计算控制输出和预测结果。
     *
     * @param state 当前状态，包含车辆的位置、速度、航向等信息
     * @param linearize_point 线性化点，用于在优化过程中线性化车辆动力学模型
     * @param track_input 参考轨迹，包含一系列的位置、速度、航向等信息
     * @param out 控制输出，包含计算出的转向角和加速度
     * @param pred_out 预测结果，包含预测的车辆状态序列（可选）
     */
    void MPCController::Update(const State &state, const State &linearize_point, const std::vector<PoseStamped> &track_input, ControlOutput *out, std::vector<State> *pred_out)
    {
        /*
            Variable:
                [s0 s1 s2 s3 ...](t=0) [s0 s1 s2 s3 ...](t=1) ... [c0 c1 ...](t=0)
        */
        const int dim_state = 4;
        const int dim_control = 2;

        /*
            Sub-functions
        */
        auto state_var_idx = [&] (int t, int id) {
            return t * dim_state + id;
        };
        auto control_var_idx = [&] (int t, int id) {
            return param_.pred_horizon * dim_state + std::min(t, param_.control_horizon - 1) * dim_control + id;
        };
        auto control_derivative_row_idx = [&] (int t, int id) {
            return dim_state * param_.pred_horizon + dim_control * param_.control_horizon + std::min(t, param_.control_horizon - 1) * dim_control + id;
        };
        /*auto normalize_angle = [&] (double a) {
            return fmod(fmod(a + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI;
        };*/
        auto get_beta_from_steer = [&] (double steer) {
            return std::atan(model_.l_r / (model_.l_f + model_.l_r) * std::tan(steer));
        };
        auto get_steer_from_beta = [&] (double beta) {
            return std::atan((model_.l_f + model_.l_r) / model_.l_r * std::tan(beta));
        };

        /*
            Setup and solve QP
        */

        int n = dim_state * param_.pred_horizon + dim_control * param_.control_horizon; //number of variables
        int m = n + dim_control * param_.control_horizon; //number of constraints

        c_float dt = param_.dt;

        qp_.initialize(n, m);

        /*
            initialize constraint matrix
        */

        //create initial state constraints
        for(int i = 0; i < dim_state; i++) qp_.A_.addElement(state_var_idx(0, i), state_var_idx(0, i), 1);
        qp_.l_[0] = qp_.u_[0] = state.x;
        qp_.l_[1] = qp_.u_[1] = state.y;
        qp_.l_[2] = qp_.u_[2] = state.v;
        qp_.l_[3] = qp_.u_[3] = state.yaw;

        c_float linearize_beta = get_beta_from_steer(linearize_point.steer_angle);

        //create state model constraints
        for(int t = 1; t < param_.pred_horizon; t++) {
            //adaptive MPC

            //x
            qp_.A_.addElement(state_var_idx(t, 0), state_var_idx(t, 0), -1);
            qp_.A_.addElement(state_var_idx(t, 0), state_var_idx(t - 1, 0), 1);

            qp_.A_.addElement(state_var_idx(t, 0), state_var_idx(t - 1, 2), dt * std::cos(linearize_point.yaw + linearize_beta));
            qp_.A_.addElement(state_var_idx(t, 0), state_var_idx(t - 1, 3), dt * -linearize_point.v * std::sin(linearize_point.yaw + linearize_beta));
            qp_.l_[state_var_idx(t, 0)] = qp_.u_[state_var_idx(t, 0)] =   -(dt *  linearize_point.v * std::sin(linearize_point.yaw + linearize_beta) * (linearize_point.yaw + linearize_beta));

            //y
            qp_.A_.addElement(state_var_idx(t, 1), state_var_idx(t, 1), -1);
            qp_.A_.addElement(state_var_idx(t, 1), state_var_idx(t - 1, 1), 1);

            qp_.A_.addElement(state_var_idx(t, 1), state_var_idx(t - 1, 2), dt * std::sin(linearize_point.yaw + linearize_beta));
            qp_.A_.addElement(state_var_idx(t, 1), state_var_idx(t - 1, 3), dt * linearize_point.v * std::cos(linearize_point.yaw + linearize_beta));
            qp_.l_[state_var_idx(t, 1)] = qp_.u_[state_var_idx(t, 1)] =  -(-dt * linearize_point.v * std::cos(linearize_point.yaw + linearize_beta) * (linearize_point.yaw + linearize_beta));

            //v
            qp_.A_.addElement(state_var_idx(t, 2), state_var_idx(t, 2), -1);
            qp_.A_.addElement(state_var_idx(t, 2), state_var_idx(t - 1, 2), 1);
            qp_.A_.addElement(state_var_idx(t, 2), control_var_idx(t - 1, 1), dt);
            qp_.l_[state_var_idx(t, 2)] = qp_.u_[state_var_idx(t, 2)] = 0;

            //yaw
            qp_.A_.addElement(state_var_idx(t, 3), state_var_idx(t, 3), -1);
            qp_.A_.addElement(state_var_idx(t, 3), state_var_idx(t - 1, 3), 1);
            qp_.A_.addElement(state_var_idx(t, 3), state_var_idx(t - 1, 2), dt * std::sin(linearize_beta) / model_.l_r);
            qp_.A_.addElement(state_var_idx(t, 3), control_var_idx(t - 1, 0), dt * linearize_point.v / model_.l_r * std::cos(linearize_beta));
            qp_.l_[state_var_idx(t, 3)] = qp_.u_[state_var_idx(t, 3)] =    -(-dt * linearize_point.v / model_.l_r * std::cos(linearize_beta) * linearize_beta);
        }

        //create control output constraints
        for(int t = 0; t < param_.control_horizon; t++) {
            for(int i = 0; i < dim_control; i++) qp_.A_.addElement(control_var_idx(t, i), control_var_idx(t, i), 1);
            
            qp_.l_[control_var_idx(t, 0)] = get_beta_from_steer( - hard_constraint_.max_steer);
            qp_.u_[control_var_idx(t, 0)] = get_beta_from_steer( hard_constraint_.max_steer);

            qp_.l_[control_var_idx(t, 1)] = hard_constraint_.min_accel;
            qp_.u_[control_var_idx(t, 1)] = hard_constraint_.max_accel;
        }

        //create control derivative constraints
        //integral average approximation
        double max_delta_beta  = dt * hard_constraint_.max_steer_rate * ((get_beta_from_steer(hard_constraint_.max_steer) - get_beta_from_steer(0)) / (hard_constraint_.max_steer - 0));
        double max_delta_accel = dt * hard_constraint_.max_jerk;

        for(int i = 0; i < dim_control; i++) {
            qp_.A_.addElement(control_derivative_row_idx(0, i), control_var_idx(0, i), 1);
        }

        c_float state_beta = get_beta_from_steer(state.steer_angle);
        qp_.l_[control_derivative_row_idx(0, 0)] = state_beta - max_delta_beta;
        qp_.u_[control_derivative_row_idx(0, 0)] = state_beta + max_delta_beta;

        qp_.l_[control_derivative_row_idx(0, 1)] = state.accel - max_delta_accel;
        qp_.u_[control_derivative_row_idx(0, 1)] = state.accel + max_delta_accel;

        for(int t = 1; t < param_.control_horizon; t++) {
            for(int i = 0; i < dim_control; i++) {
                qp_.A_.addElement(control_derivative_row_idx(t, i), control_var_idx(t, i),      1);
                qp_.A_.addElement(control_derivative_row_idx(t, i), control_var_idx(t - 1, i), -1);
            }

            qp_.l_[control_derivative_row_idx(t, 0)] = -max_delta_beta;
            qp_.u_[control_derivative_row_idx(t, 0)] =  max_delta_beta;

            qp_.l_[control_derivative_row_idx(t, 1)] = -max_delta_accel;
            qp_.u_[control_derivative_row_idx(t, 1)] =  max_delta_accel;
        }

        //create cost function
        //pred horizon
        for(int t = 1; t < param_.pred_horizon; t++) {
            PoseStamped pose = track_input[t - 1];

            /*
                Fix yaw +-2pi for QP
            */
            if(std::fabs((pose.yaw + M_PI * 2.0) - state.yaw) < std::fabs(pose.yaw - state.yaw))
                pose.yaw += M_PI * 2.0;

            if(std::fabs((pose.yaw - M_PI * 2.0) - state.yaw) < std::fabs(pose.yaw - state.yaw))
                pose.yaw -= M_PI * 2.0;

            //position
            qp_.P_.addElement(state_var_idx(t, 0), state_var_idx(t, 0), cost_weight_.w_position);
            qp_.q_[state_var_idx(t, 0)] = -cost_weight_.w_position * pose.x;

            qp_.P_.addElement(state_var_idx(t, 1), state_var_idx(t, 1), cost_weight_.w_position);
            qp_.q_[state_var_idx(t, 1)] = -cost_weight_.w_position * pose.y;

            //angle
            qp_.P_.addElement(state_var_idx(t, 3), state_var_idx(t, 3), cost_weight_.w_angle);
            qp_.q_[state_var_idx(t, 3)] = -cost_weight_.w_angle * pose.yaw;

            //velocity
            qp_.P_.addElement(state_var_idx(t, 2), state_var_idx(t, 2), cost_weight_.w_velocity);
            qp_.q_[state_var_idx(t, 2)] = -cost_weight_.w_velocity * pose.v;
        }

        //control horizon
        for(int t = 0; t < param_.control_horizon; t++) {
            //accel
            qp_.P_.addElement(control_var_idx(t, 0), control_var_idx(t, 0), cost_weight_.w_accel);

            //steer
            qp_.P_.addElement(control_var_idx(t, 1), control_var_idx(t, 1), cost_weight_.w_steer);
        }

        //solve qp
        OSQPSolution *solution = qp_.solve(&out->error_code);

        out->steer = get_steer_from_beta(solution->x[control_var_idx(0, 0)]);
        out->accel = solution->x[control_var_idx(0, 1)];

        //output predictive result
        if(pred_out != nullptr) {
            pred_out->clear();

            for(int t = 0; t < param_.pred_horizon; t++) {
                State pred_state;
                pred_state.x = solution->x[state_var_idx(t, 0)];
                pred_state.y = solution->x[state_var_idx(t, 1)];
                pred_state.v = solution->x[state_var_idx(t, 2)];
                pred_state.yaw = solution->x[state_var_idx(t, 3)];

                pred_out->push_back(pred_state);
            }
        }
    }

    /**
     * @brief 模拟控制器模型的状态更新
     *
     * 根据当前状态和未来控制输入，计算并更新下一时刻的车辆状态
     *
     * @param state 当前车辆状态
     * @param next 用于存储计算得到的下一时刻的车辆状态
     */
    void MPCController::ControllerModelSimulate(const State &state, State *next)
    {
        double dt = param_.dt;
        double beta = std::atan(model_.l_r / (model_.l_f + model_.l_r) * std::tan(state.steer_angle));

        next->x   = state.x + dt * state.v * std::cos(state.yaw + beta);
        next->y   = state.y + dt * state.v * std::sin(state.yaw + beta);
        next->yaw = state.yaw + dt * state.v / model_.l_r * std::sin(beta);
        next->v   = state.v + dt * state.accel;

        next->accel = state.accel;
        next->steer_angle = state.steer_angle;
    }

    void MPCController::IterativeUpdate(const State &state, const std::vector<PoseStamped> &track_input, int iterations, double threshold, ControlOutput *out, std::vector<State> *pred_out)
    {
        State state_with_new_control_input = state;

        int num_iterations = 0;
        for(int i = 0; i < iterations; i++) {
            num_iterations++;

            State linearize_point;
            ControllerModelSimulate(state_with_new_control_input, &linearize_point);

            Update(state, linearize_point, track_input, out, pred_out);

            double delta_output = 
                std::fabs(state_with_new_control_input.accel - out->accel) + 
                std::fabs(state_with_new_control_input.steer_angle - out->steer);

            if(delta_output < threshold) break;

            state_with_new_control_input.accel       = out->accel;
            state_with_new_control_input.steer_angle = out->steer;
        }
    }

    bool MPCController::GetReferenceTrajectory(Path ref_path)
    {
        // if (ref_path.poses.size() == 0) {
        //     return false;
        // }
        path_adapter_->LoadReferenceTrajectory(ref_path);
        return true;
    }

    void MPCController::RefPathCallback(const Path::ConstSharedPtr msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("mpc_controller"), "Received global path");
        // 更新全局路径
        ref_path_ = *msg;
    }

} // namespace Controller