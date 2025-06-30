manage.h

// 
#pragma once
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include "Rob.h"
#include "PISMP.h"

class MotionController {
public:
    MotionController(Robot* robot_ptr);
    // ~MotionController();

    void start();
    void stop();

    void produce_dx();  // dx由模型推理出来

    Eigen::Vector2d infer_dx(const Eigen::Vector2d& input_vec);

    void producer();  // 模拟从模型获取 dx -> dq
    void consumer();  // 机械臂执行 dq（通过 Robot）

    std::queue<Eigen::Matrix<double, 6, 1>> dq_queue;

    std::mutex robot_mtx;  // 专门保护 robot
    std::mutex queue_mtx;  // 专门保护 dq_queue
    
    std::condition_variable cv;
    bool stop_flag = false;

    std::thread producer_thread;
    std::thread consumer_thread;

    Robot* robot;
    PISMP  pismp;

};


manage.cpp

// motion_controller.cpp
#include "manage.h"

MotionController::MotionController(Robot* robot_ptr)
    : robot(robot_ptr), pismp() {}

void MotionController::start() {
    stop_flag = false;
    producer_thread = std::thread(&MotionController::producer, this);
    consumer_thread = std::thread(&MotionController::consumer, this);
}

void MotionController::stop() {
    {
        std::lock_guard<std::mutex> lock(mtx);
        stop_flag = true;
    }
    cv.notify_all();
    producer_thread.join();
    consumer_thread.join();
}

void MotionController::produce_dx() {
    // robot->FK();             // 计算实时位置
    // robot->compute_J_inv();  // 计算雅可比矩阵
    // Eigen::Matrix<double, 2, 1> x = robot->position.head<2>();  // 两个维度
    // Eigen::Vector2d dx2 = infer_dx(x);  // 2维推理结果
    // Eigen::Matrix<double, 6, 1> dx6 = Eigen::Matrix<double, 6, 1>::Zero();  // 6维全零向量
    // dx6.head<2>() = dx2;  // 把前两维赋值为 dx2
    // Eigen::Matrix<double, 6, 1> dq = robot->J_inv * dx6;

    // {
    //     std::lock_guard<std::mutex> lock(mtx);
    //     dq_queue.push(dq);
    // }
    // cv.notify_one();

    {
        std::lock_guard<std::mutex> lock(robot_mtx);
        robot->FK();
        robot->compute_J_inv();
        Eigen::Matrix<double, 2, 1> x = robot->position.head<2>();
        Eigen::Vector2d dx2 = infer_dx(x);
        Eigen::Matrix<double, 6, 1> dx6 = Eigen::Matrix<double, 6, 1>::Zero();
        dx6.head<2>() = dx2;
        dq = robot->J_inv * dx6;
    }

    {
        std::lock_guard<std::mutex> lock(queue_mtx);
        dq_queue.push(dq);
    }

    cv.notify_one();
}

Eigen::Vector2d MotionController::infer_dx(const Eigen::Vector2d& input_vec) {
    // 将 Eigen::Vector2d 转换为 1x2 的 torch::Tensor
    torch::Tensor input_tensor = torch::from_blob(
        const_cast<double*>(input_vec.data()),
        {1, 2},
        torch::TensorOptions().dtype(torch::kDouble)).clone();  // 避免悬挂指针

    // 模型推理
    torch::Tensor output = pismp.infer(input_tensor);  // output shape: [1, 2]
    output = output.squeeze();  // 变为 [2]

    // 转换为 Eigen::Vector2d
    Eigen::Vector2d dx;
    for (int i = 0; i < 2; ++i) {
        dx(i) = output[i].item<double>();
    }

    return dx;
}

void MotionController::producer() {
    while (!stop_flag) {
        produce_dx();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MotionController::consumer() {
    while (!stop_flag) {
        std::unique_lock<std::mutex> lock(queue_mtx);
        cv.wait(lock, [&]() { return !dq_queue.empty() || stop_flag; });

        if (stop_flag) break;

        Eigen::Matrix<double, 6, 1> dq = dq_queue.front();
        // robot->ode(dq); // 将dq传进去 进行递归循环处理
        dq_queue.pop();
        lock.unlock();
        
        {
            std::lock_guard<std::mutex> robot_lock(robot_mtx);  // 加锁访问 robot
            robot->ode(dq);  // 用 dq 更新 robot 状态
        }

        // robot->applyDeltaQ(dq);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟执行耗时
    }
}

