// RobotArm.h
#pragma once
#include <vector>
#include <Eigen/Dense>


constexpr int JOINT_NUM = 6;
constexpr double PI = 3.141592653589793;


class Robot {
public:
    Robot();
    Robot(Eigen::Matrix<double, 6, 1> Q0);
    // 设置关节角度（单位：弧度）
    void setJointAngles(const Eigen::Matrix<double, 6, 1> Q );

    // 获取末端位姿（正运动学） 和雅可比矩阵
    void FK();
    void compute_J_inv(bool use_damped = false, double lambda = 0.01);
    void ode(const Eigen::Matrix<double, 6, 1>& dq);
    
    Eigen::Matrix<double, JOINT_NUM, 1> Q0;
    Eigen::Matrix<double, 3, 1> X0;
    Eigen::Matrix<double, JOINT_NUM, 1> JointQ;     // 关节位置（角度）
    Eigen::Matrix<double, JOINT_NUM, 1> JointdQ;    // 关节速度
    Eigen::Matrix<double, 3, 1> position;           // 末端位置
    Eigen::Matrix<double, 3, 3> rotation;           // 末端姿态
    Eigen::Matrix4d end_effector_pose;              // 末端执行器位姿，4x4齐次变换矩阵
    Eigen::Matrix<double, 6, JOINT_NUM> J;          // 雅可比矩阵（6行n列）
    Eigen::Matrix<double, JOINT_NUM, 6> J_inv;      // 雅可比伪逆

    double dt=0.001;


};

