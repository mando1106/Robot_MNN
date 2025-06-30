#include "manage.h"
#include <chrono>
#include <thread>
#include <iostream>

manage::manage(Robot* robot_ptr)
    : robot(robot_ptr), pismp(), stop_flag(false) 
    {
        // 设置初始位置信息
        x_     << 0, 0.8;           // 末端位置
        x_true << 0, 0;
        dt = 1 ;
        stop_flag = false;
        // 启动新线程
        control_thread = std::thread(&manage::control_loop, this);
    }

manage::~manage() {
    stop();
}

void manage::stop() {
    stop_flag = true;

    // 等待线程结束
    if (control_thread.joinable()) {
        control_thread.join();
    }
    std::cout << "Control loop stopped 1." << std::endl;
}

void manage::control_loop() {
    std::cout << "Control loop started." << std::endl;
    double max_time = 0.0, min_time = 1e9;
    int i=0;
    double total_infer_time_ms = 0.0;
    int max_index = -1;
    while (!stop_flag) {
        // 这个版本没有 机器人的相关信息
                // 计时开始
        auto t_start = std::chrono::high_resolution_clock::now();
        dx_     = pismp.infer_dx(x_);
        auto t_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> infer_duration = t_end - t_start;
        total_infer_time_ms += infer_duration.count();  // 累加推理时间

        // double t_ms = infer_duration.count(); 
        // if (t_ms > max_time) {
        //     max_index = i;  // 记录出现最大时间的索引
        // }
        // max_time = std::max(max_time, t_ms);
        // min_time = std::min(min_time, t_ms);

        if (i >= 1) {  // 忽略前10次 warmup
            double t_ms = infer_duration.count(); 
            if (t_ms > max_time) {
                max_index = i;  // 记录出现最大时间的索引
            }
            max_time = std::max(max_time, t_ms);
            min_time = std::min(min_time, t_ms);
        }


        delta_x = dt * dx_;
        x_ = x_ + delta_x;

        //真实需要前进的步伐
        x_true = x_true + delta_x;

        X_.push_back(x_);  // 保存当前x_
        X_true.push_back(x_true);  // 保存当前x_

        if (++i > 2000) break;
    }

    double avg_infer_time = total_infer_time_ms / static_cast<double>(i);

    std::cout << "Control loop stopped." << std::endl;
    std::cout << "min_time: " << min_time << " ms,"<< " max_time: " << max_time << " ms," <<" max_time_index: " << max_index   << std::endl;
    std::cout << "Average inference time per step: " << avg_infer_time << " ms" << std::endl;
}


