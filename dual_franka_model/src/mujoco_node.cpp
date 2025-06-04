#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <cstddef>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;

class MujocoSimNode : public rclcpp::Node {
    public:
        MujocoSimNode() : Node("mujoco_sim_node") {
            // Load Model
            char error[1000] = "could not load model";
            std::string package_path = ament_index_cpp::get_package_share_directory("dual_franka_model");
            std::string model_path = package_path + "/" + this->declare_parameter<std::string>("model_path", "mjx_panda.xml");
            model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
            if (!model_) {
                RCLCPP_FATAL(this->get_logger(), "Failed to load model: %s", error);
                rclcpp::shutdown();
                return;
            }
    
            data_ = mj_makeData(model_);
            if (!data_) {
                RCLCPP_FATAL(this->get_logger(), "Failed to make data for model");
                rclcpp::shutdown();
                return;
            }
    
            // Initialize GLFW
            if (!glfwInit()) {
                RCLCPP_FATAL(this->get_logger(), "Failed to initialize GLFW");
                rclcpp::shutdown();
                return;
            }
    
            // Create GLFW window
            window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulation", nullptr, nullptr);
            if (!window_) {
                RCLCPP_FATAL(this->get_logger(), "Failed to create GLFW window");
                glfwTerminate();
                rclcpp::shutdown();
                return;
            }
            glfwMakeContextCurrent(window_);
            glfwSwapInterval(1);
    
            // Initialize visualization
            mjv_defaultCamera(&cam_);
            mjv_defaultOption(&opt_);
            mjr_defaultContext(&context_);
            mjr_makeContext(model_, &context_, mjFONTSCALE_150);
            mjv_defaultScene(&scene_);
            mjv_makeScene(model_, &scene_, 1000);
    
            // Set up the camera
            cam_.type = mjCAMERA_FREE;
            cam_.lookat[0] = 0.0;
            cam_.lookat[1] = 0.0;
            cam_.lookat[2] = 0.5;
            cam_.distance = 2.0;
            cam_.azimuth = 90;
            cam_.elevation = -20;
    
            // ROS publisher
            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
    
            // Create a timer loop
            sim_timer_ = this->create_wall_timer(10ms, std::bind(&MujocoSimNode::step_simulation, this));
            RCLCPP_INFO(this->get_logger(), "Mujoco sim node started");
        }
    
        ~MujocoSimNode() {
            if (data_) mj_deleteData(data_);
            if (model_) mj_deleteModel(model_);
            mjr_freeContext(&context_);
            mjv_freeScene(&scene_);
            glfwDestroyWindow(window_);
            glfwTerminate();
        }
    
    private:
        void step_simulation() {
          
            if (glfwWindowShouldClose(window_)) {
                rclcpp::shutdown();
                return;
            }
            
    
            // Publish joint states
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = this->get_clock()->now();
            for (int i = 0; i < model_->nq; i++) { // number of generalized coordinates
                msg.name.push_back(model_->names + model_->name_jntadr[i]);
                msg.position.push_back(data_->qpos[i]);
            }
            joint_pub_->publish(msg);
    
            // Render the scene
            int width, height;
            glfwGetFramebufferSize(window_, &width, &height);
            mjrRect viewport = {0, 0, width, height};
            mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);
            mjr_render(viewport, &scene_, &context_);
            glfwSwapBuffers(window_);
            glfwPollEvents();
            // Eigen::VectorXd desired_position;
            // desired_position <<1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0; // Example desired position
            for (int i = 0; i < model_->nu; ++i)
                data_->ctrl[i] =1.0;
            //std::cerr<<"Number of actuators: "<<model_->nu<<std::endl;
    
            // Step the simulation
            mj_step(model_, data_);
        }
    
        mjModel* model_ = nullptr;
        mjData* data_ = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        rclcpp::TimerBase::SharedPtr sim_timer_;
    
        // Visualization components
        GLFWwindow* window_ = nullptr;
        mjvCamera cam_;
        mjvOption opt_;
        mjvScene scene_;
        mjrContext context_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<MujocoSimNode> node = std::make_shared<MujocoSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}