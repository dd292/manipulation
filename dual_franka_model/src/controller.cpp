#include "controller.hpp"
#include <iostream>

bool Controller::compute_ik(const Eigen::Vector3d & target_pos, int gripper_id, double threshold, int max_iterations)
{
    Eigen::Vector3d gripper_pos(data_->xpos[3 * gripper_id], data_->xpos[3 * gripper_id + 1], data_->xpos[3 * gripper_id + 2]);
    Eigen::Vector3d delta_pos = target_pos - gripper_pos;

    int iteration = 0;
    while (delta_pos.norm() > threshold && iteration < max_iterations) {
        // Compute the Jacobian
        Eigen::MatrixXd jacobian(3, model_->nu);
        mj_jac(model_, data_, jacobian.data(), nullptr, data_->xpos + 3 * gripper_id, gripper_id);

        // Compute joint velocity using pseudo-inverse
        Eigen::VectorXd joint_velocity = jacobian.completeOrthogonalDecomposition().solve(delta_pos);

        // Apply joint velocity to control inputs
        // std::cerr << "Control inputs before update:"<<std::endl;
        // for (int i = 0; i < model_->nu; ++i) {
        //     std::cerr << "Ctrl[" << i << "]: " << data_->ctrl[i] << std::endl;
        // }
        for (int i = 0; i < model_->nu; ++i) {
            data_->ctrl[i] = data_->qpos[i] + joint_velocity[i] * model_->opt.timestep;
        }

        // Step the simulation
        mj_step(model_, data_);
        gripper_pos = Eigen::Vector3d(data_->xpos[3 * gripper_id], data_->xpos[3 * gripper_id + 1], data_->xpos[3 * gripper_id + 2]);
        delta_pos = target_pos - gripper_pos;
        iteration++;
    }

    return delta_pos.norm() <= threshold;
}