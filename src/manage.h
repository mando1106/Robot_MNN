#pragma once
#include "Rob.h"
#include "PISMP.h"
#include <thread>      // 添加线程支持
#include <atomic>      // 用于线程安全的 stop_flag
#include <Eigen/Dense>

class manage {
public:
    manage(Robot* robot_ptr);
    ~manage();
    void stop();       // 停止控制器
    void control_loop(); // 控制循环函数

    
    std::vector<Eigen::Matrix<double, 2, 1>> X_;      // 保存所有推理x;
    std::vector<Eigen::Matrix<double, 2, 1>> X_true ; //保存所有 真实X_true;


private:
    Robot* robot;
    PISMP pismp;
    
    std::atomic<bool> stop_flag;     // 更安全的线程标志
    std::thread control_thread;      // 控制线程

    Eigen::Matrix<double, 2, 1> x_;           // 末端位置
    Eigen::Matrix<double, 2, 1> delta_x;           // 末端位置
    Eigen::Matrix<double, 2, 1> dx_;           // 末端姿态
    Eigen::Matrix<double, 2, 1> x_true;           // 末端位置
    Eigen::Matrix<double, 2, 1> dx_true;           // 末端姿态
    double dt;
};
