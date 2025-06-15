#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <Eigen/Dense>
#include <mujoco/mujoco.h>


class Controller {
    public:
        Controller(mjModel* model, mjData*data) :model_(model), data_(data) {}
        //Compute IK to move the gripper to the target position
        bool compute_ik(const Eigen::Vector3d & target_pos, int gripper_id, double threshold, int max_iterations);
    private:
        mjModel* model_; // Pointer to the Mujoco model
        mjData* data_;   // Pointer to the Mujoco data
};
#endif