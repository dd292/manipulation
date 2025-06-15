#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <vector>
#include <iostream>
#include <mujoco/mujoco.h>

struct Point3D {
    double x, y, z;
};

class Planner{
    public:
        Planner(double step_size, int max_iterations, mjModel* model, mjData* data): step_size_(step_size), max_iterations_(max_iterations), model_(model), data_(data) {}

        std::vector<Point3D> rrt(const Point3D&start, const Point3D& goal);
    private:
        double step_size_;
        int max_iterations_;
        mjModel* model_; // Pointer to the Mujoco model
        mjData* data_;   // Pointer to the Mujoco data

        bool is_collision_free(const Point3D& point);
};
#endif
