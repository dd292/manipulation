#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>  
#include <cstddef> 
#include<mujoco/mujoco.h>
#include<GLFW/glfw3.h>
#include<chrono>
#include <simulate.h>
#include "ament_index_cpp/get_package_share_directory.hpp"


using namespace std::chrono_literals;

class MujocoSimNode: public rclcpp::Node{
    public:
        MujocoSimNode():Node("mujoco_sim_node")
        {
            //Load Model
            char error[1000]= "could not load model";
            std::string package_path = ament_index_cpp::get_package_share_directory("dual_franka_model");
            std::string model_path = package_path + "/" + this->declare_parameter<std::string> ("model_path", "mjx_panda.xml");
            model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
            if (!model_){
                RCLCPP_FATAL(this->get_logger(), "Failed to load model :%s", error);
                rclcpp::shutdown();
                return;
            }

            data_ = mj_makeData(model_);
            data_ = mj_makeData(model_);
            if (!data_) {
                RCLCPP_FATAL(this->get_logger(), "Failed to make data for model");
                rclcpp::shutdown();
                return;
            }
            // Setup camera and visualization options
            cam_ = std::make_unique<mjvCamera>();
            opt_ = std::make_unique<mjvOption>();
            perturb_ = std::make_unique<mjvPerturb>();

            m_sim = std::make_unique<mj::Simulate>(
                std::make_unique<UIAdapterWithCppCallback>(
                    std::bind(&MujocoSimNode::KeyCallback, this, std::placeholders::_1)),
                cam_.get(), opt_.get(), perturb_.get(), false);
            
            //ROS publisher
            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);

            //Create a timer loop
            sim_timer_= this->create_wall_timer(10ms, std::bind(&MujocoSimNode::step_simulation, this));
            RCLCPP_INFO(this->get_logger(), "Mujoco sim node started");

        }
        ~MujocoSimNode(){
            if (data_) mj_deleteData(data_);
            if (model_) mj_deleteModel(model_);
        }
    private:
        void step_simulation()
        {
            if (glfwWindowShouldClose(window_)) {
                rclcpp::shutdown();
                return;
            }
            mj_step(model_, data_);
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = this->get_clock()->now();
            for (int i=0; i<model_->nq; i++) //number of generalized coordinates
            {
                msg.name.push_back(model_->names+model_->name_jntadr[i]);
                msg.position.push_back(data_->qpos[i]);

            }
            joint_pub_->publish(msg);

            // Render simulation
            if (m_sim_) {
                m_sim_->Render(*model_, *data_);
            }
        }
        mjModel* model_ = nullptr;
        mjData* data_ = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        rclcpp::TimerBase::SharedPtr sim_timer_;
        std::unique_ptr<mj::Simulate> m_sim_;
        std::unique_ptr<mjvCamera> cam_;
        std::unique_ptr<mjvOption> opt_;
        std::unique_ptr<mjvPerturb> perturb_;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MujocoSimNode> node = std::make_shared<MujocoSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}