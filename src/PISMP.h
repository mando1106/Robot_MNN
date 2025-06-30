#pragma once
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <cstring>

class PISMP {
public:
    std::unique_ptr<MNN::Interpreter> net_;
    MNN::Session* session_;
    Eigen::Matrix<double, 2, 1> x_min;
    Eigen::Matrix<double, 2, 1> x_max;
    Eigen::Matrix<double, 2, 1> max_state_derivative;

    PISMP() {
        // 示例缩放值（你可以根据实际设置）
        // x_min << -14.90530847 , -7.62892629;
        // x_max <<  51.43349777, 55.92682467;
        max_state_derivative << 0.0076, 0.0051;

        try {
            net_.reset(MNN::Interpreter::createFromFile("PISMP_model.mnn"));

            MNN::ScheduleConfig config;
            config.numThread = 1;
            session_ = net_->createSession(config);

            std::cout << "PISMP_MNN initialized." << std::endl;
        } catch (...) {
            std::cerr << "Failed to initialize MNN interpreter." << std::endl;
            throw;
        }
    }

    Eigen::Vector2d infer_dx(const Eigen::Vector2d& input_vec) {
        // 构造输入数据
        std::vector<float> input_data = {
            static_cast<float>(input_vec(0)),
            static_cast<float>(input_vec(1))
        };

        // 获取输入 Tensor 并赋值
        MNN::Tensor* input_tensor = net_->getSessionInput(session_, nullptr);
        MNN::Tensor input_host(input_tensor, MNN::Tensor::CAFFE);
        std::memcpy(input_host.buffer().host, input_data.data(), input_data.size() * sizeof(float));
        input_tensor->copyFromHostTensor(&input_host);

        // 推理
        net_->runSession(session_);

        // 获取输出 Tensor
        MNN::Tensor* output_tensor = net_->getSessionOutput(session_, nullptr);
        MNN::Tensor output_host(output_tensor, MNN::Tensor::CAFFE);
        output_tensor->copyToHostTensor(&output_host);

        // 读取结果
        float* output_data = output_host.host<float>();
        Eigen::Vector2d dx;
        dx(0) = static_cast<double>(output_data[0]);
        dx(1) = static_cast<double>(output_data[1]);

        return denormalize_derivative(dx);
    }

    // --------- 归一化与反归一化工具函数 ---------

    Eigen::Matrix<double, 2, 1> normalize_state(const Eigen::Matrix<double, 2, 1>& state) {
        return (((state - x_min).array() / (x_max - x_min).array()) - 0.5) * 2.0;
    }

    Eigen::Matrix<double, 2, 1> denormalize_state(const Eigen::Matrix<double, 2, 1>& state) {
        return (((state.array() / 2.0) + 0.5) * (x_max - x_min).array()) + x_min.array();
    }

    Eigen::Vector2d denormalize_state(const Eigen::Vector2d& state, double scale) const {
        Eigen::Vector2d temp_min = Eigen::Vector2d::Constant(-scale);
        Eigen::Vector2d temp_max = Eigen::Vector2d::Constant(scale);
        return (((state.array() / 2.0) + 0.5) * (temp_max - temp_min).array()) + temp_min.array();
    }

    Eigen::Matrix<double, 2, 1> normalize_derivative(const Eigen::Matrix<double, 2, 1>& dx_t) {
        return dx_t.array() / max_state_derivative.array();
    }

    Eigen::Matrix<double, 2, 1> denormalize_derivative(const Eigen::Matrix<double, 2, 1>& dx_t_normalized) {
        return dx_t_normalized.array() * max_state_derivative.array();
    }
};
