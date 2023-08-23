/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu_client.hpp"

class ImuTest: public rclcpp::Node
{
public:
    ImuTest() : Node("imu_test")
    {
        _subscription = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "topic msg: <%u, %f, %f, %f, %f, %f, %f>",
                    msg->header.stamp,
                    msg->linear_acceleration.x,
                    msg->linear_acceleration.y,
                    msg->linear_acceleration.z,
                    msg->angular_velocity.x,
                    msg->angular_velocity.y,
                    msg->angular_velocity.z
                );
            }
        );
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _subscription;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
