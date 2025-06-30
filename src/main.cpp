#include"Rob.h"
#include"manage.h"
#include"Plotter.h"
#include<iostream>



int main(){
    Robot robot;
    Eigen::Matrix<double, 6, 1> q;
    q << 0, -PI/4, PI/2, -PI/4, PI/2, 0;
    robot.setJointAngles(q);
    robot.FK();
    robot.compute_J_inv();

    // std::cout << "Position:\n" << robot.position.transpose() << std::endl;
    // std::cout << "Tn:\n" << robot.end_effector_pose << std::endl;
    // std::cout << "Jacobian:\n" << robot.J << std::endl;
    // std::cout << "J_inv:\n" << robot.J_inv << std::endl;

    manage manager(&robot);


    std::cout << "manage started. Running ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5)); 

     // 停止控制器，关闭线程
    manager.stop();

    std::vector<std::vector<double>> X_plot(2);  // 每个关节的轨迹
    std::vector<std::vector<double>> X_true_plot(2);  // 每个关节的轨迹


    for (const auto& q : manager.X_) {
        for (int j = 0; j < 2; ++j) {
            X_plot[j].push_back(q(j));
        }       
    }
    for (const auto& q : manager.X_true) {
        for (int j = 0; j < 2; ++j) {
            X_true_plot[j].push_back(q(j));
        }       
    }
    // std::cout << "X_plot values:" << std::endl;
    // for (size_t i = 0; i < X_plot[0].size(); ++i) {
    //     std::cout << "[" << X_plot[0][i] << ", " << X_plot[1][i] << "]" << std::endl;
    // }

    std::cout << "PLoting ..." << std::endl;

    Plotter::plot2D(X_plot[0], X_plot[1]);
    Plotter::plot2D(X_true_plot[0], X_true_plot[1]);
    Plotter::show(); // 显示所有轨迹图

    std::cout << "Manage stopped. Exiting program." << std::endl;


    return 0;
}