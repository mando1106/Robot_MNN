#include"Rob.h"

#include <cmath>

// 定义DH参数

const double a[6] = {0, 0, -0.2435, -0.211, 0, 0};
const double d[6] = {0.1435, 0, 0, 0.086, 0.086, 0.0725};
const double alpha[6] = {0, PI / 2, PI, PI, PI / 2, -PI / 2};
const double theta[6] = {0, -PI / 2, 0, -PI / 2, 0, 0};



Robot::Robot()
    {
        JointQ.setZero();
        JointdQ.setZero();
        position.setZero();
        rotation.setIdentity();
        end_effector_pose.setIdentity();
        J.setZero();
        J_inv.setZero();
    }

Robot::Robot(Eigen::Matrix<double, 6, 1> Q0)
    {
        this->JointQ = Q0;
        this->Q0 = Q0;
        FK();
        this->X0=this->position;
    }

void Robot::setJointAngles(Eigen::Matrix<double, 6, 1> Q) {
    this->JointQ = Q;   
}


void Robot::FK() {
    using namespace Eigen;

    Matrix4d Tn = Matrix4d::Identity();
    Matrix<double, 3, 6> P;    // 位姿向量
    Matrix<double, 3, 6> Jw;   // 角速度部分
    Matrix<double, 3, 6> Jv;   // 线速度部分

    for (int i = 0; i < 6; ++i) {
        double joint = this->JointQ[i] + theta[i];
        double ca = cos(alpha[i]), sa = sin(alpha[i]);
        double cj = cos(joint), sj = sin(joint);

        // Denavit-Hartenberg变换矩阵
        Matrix4d A;
        A << cj, -sj, 0, a[i],
             sj * ca, cj * ca, -sa, -d[i] * sa,
             sj * sa, cj * sa, ca, d[i] * ca,
             0, 0, 0, 1;

        Tn = Tn * A;

        // 提取位置和z轴方向
        P.col(i) = Tn.block<3,1>(0,3);  // 位姿
        Jw.col(i) = Tn.block<3,1>(0,2); // 当前z轴
    }

    Vector3d p_end = Tn.block<3,1>(0,3);
    for (int i = 0; i < 6; ++i) {
        Vector3d diff = p_end - P.col(i);
        Jv.col(i) = Jw.col(i).cross(diff);
    }

    this->J << Jv, Jw;
    this->position = p_end;
    // 提取旋转矩阵（前3x3块）
    this->rotation = Tn.block<3,3>(0,0);
    this->end_effector_pose = Tn; 
}

void Robot::compute_J_inv(bool use_damped , double lambda ) {
    if (use_damped) {
        // 阻尼最小二乘伪逆: J⁺ = (JᵗJ + λ²I)⁻¹ Jᵗ
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J.cols(), J.cols());
        J_inv = (J.transpose() * J + lambda * lambda * I).inverse() * J.transpose();
    } else {
        // 普通伪逆: J⁺ = (JᵗJ)⁻¹ Jᵗ 或 Jᵗ (JJᵗ)⁻¹（根据维度）
            try {
                if (J.rows() <= J.cols()) {
                    J_inv = (J.transpose() * J).inverse() * J.transpose();
                } else {
                    J_inv = J.transpose() * (J * J.transpose()).inverse();
                }
            } catch (const std::exception& e) {
                throw std::runtime_error("Matrix inversion failed in compute_J_inv(): " + std::string(e.what()));
            }
    }
}

void Robot::ode(const Eigen::Matrix<double, 6, 1>& dq){
    JointQ = JointQ + dq*dt;

}
