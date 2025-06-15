#include "planner.hpp"
#include <cmath>
#include <random>


bool Planner::is_collision_free( const Point3D & point)
{
    int collision_body_id = mj_name2id(model_, mjOBJ_BODY, "collision_checker");
    if (collision_body_id ==-1)
    {
        std::cerr<<"Collision checked body not found in the model."<<std::endl;
        return false;
    }

    data_->xpos[3 * collision_body_id] = point.x;
    data_->xpos[3 * collision_body_id + 1] = point.y;
    data_->xpos[3 * collision_body_id + 2] = point.z;

    mj_forward(model_, data_);
    for (int i = 0; i < data_->ncon; ++i) {//iterate on detcted contacts
        const mjContact& contact = data_->contact[i];
        if (contact.geom1 == mj_name2id(model_, mjOBJ_GEOM, "table_top") ||
            contact.geom2 == mj_name2id(model_, mjOBJ_GEOM, "table_top") ||
            contact.geom1 == mj_name2id(model_, mjOBJ_GEOM, "banana_geom") ||
            contact.geom2 == mj_name2id(model_, mjOBJ_GEOM, "banana_geom"))
            {
                return false; // Collision detected with the table or banana
            }
    }

    return true;// consider no obstacles for now
}

std::vector<Point3D> Planner::rrt( const Point3D& start, const Point3D& goal)
{
    std::vector<Point3D> tree = {start};
    std::vector<Point3D> path;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);

    for (int i = 0; i < max_iterations_; ++i) {
        // Sample a random point
        Point3D random_point = {dis(gen), dis(gen), dis(gen)};
        if (!is_collision_free(random_point)) continue;

        // Find the nearest point in the tree
        Point3D nearest_point = tree[0];
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& point : tree) {
            double dist = std::sqrt(std::pow(point.x - random_point.x, 2) +
                                    std::pow(point.y - random_point.y, 2) +
                                    std::pow(point.z - random_point.z, 2));
            if (dist < min_dist) {
                min_dist = dist;
                nearest_point = point;
            }
        }

        // Move from the nearest point toward the random point
        double dx = random_point.x - nearest_point.x;
        double dy = random_point.y - nearest_point.y;
        double dz = random_point.z - nearest_point.z;
        double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
        Point3D new_point = {nearest_point.x + step_size_ * dx / norm,
                                nearest_point.y + step_size_ * dy / norm,
                                nearest_point.z + step_size_ * dz / norm};

        if (is_collision_free(new_point)) {
            tree.push_back(new_point);

            // Check if the goal is reached
            double goal_dist = std::sqrt(std::pow(new_point.x - goal.x, 2) +
                                            std::pow(new_point.y - goal.y, 2) +
                                            std::pow(new_point.z - goal.z, 2));
            if (goal_dist < step_size_) {
                path.push_back(goal);
                break;
            }
        }
    }

    // Backtrack to get the path
    path.insert(path.begin(), tree.begin(), tree.end());
    return path;
}