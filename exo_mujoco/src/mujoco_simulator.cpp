#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mujoco/mujoco.h>
#include <iostream>
#include <string>

class MuJoCoSimulator : public rclcpp::Node
{
public:
    MuJoCoSimulator()
    : Node("mujoco_simulator"), model_(nullptr), data_(nullptr)
    {
        // Declare and get the robot description parameter
        this->declare_parameter<std::string>("robot_description", "");
        std::string robot_description = this->get_parameter("robot_description").as_string();

        // Load the MuJoCo model from URDF string
        if (!robot_description.empty()) {
            char error[1000];
            model_ = mj_loadXML(robot_description.c_str(), nullptr, error, sizeof(error));
            if (!model_) {
                RCLCPP_ERROR(this->get_logger(), "Could not load MuJoCo model: %s", error);
                throw std::runtime_error(error);
            }

            data_ = mj_makeData(model_);
            if (!data_) {
                RCLCPP_ERROR(this->get_logger(), "Could not allocate MuJoCo data");
                throw std::runtime_error("Could not allocate MuJoCo data");
            }

            mj_resetData(model_, data_);

            // Start the simulation timer
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&MuJoCoSimulator::timer_callback, this));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Robot description parameter is empty");
            throw std::runtime_error("Robot description parameter is empty");
        }
    }

    ~MuJoCoSimulator()
    {
        if (data_) mj_deleteData(data_);
        if (model_) mj_deleteModel(model_);
    }

private:
    void timer_callback()
    {
        mj_step(model_, data_);

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "base_link";

        // Dummy IMU data for demonstration
        imu_msg.linear_acceleration.x = data_->qpos[0];
        imu_msg.linear_acceleration.y = data_->qpos[1];
        imu_msg.linear_acceleration.z = data_->qpos[2];
        imu_msg.angular_velocity.x = data_->qvel[0];
        imu_msg.angular_velocity.y = data_->qvel[1];
        imu_msg.angular_velocity.z = data_->qvel[2];

        publisher_->publish(imu_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    mjModel* model_;
    mjData* data_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto simulator = std::make_shared<MuJoCoSimulator>();
        rclcpp::spin(simulator);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
