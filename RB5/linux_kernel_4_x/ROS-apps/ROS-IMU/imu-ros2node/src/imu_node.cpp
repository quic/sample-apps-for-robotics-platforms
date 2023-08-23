/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <sys/sysinfo.h>
#include <time.h>
#include "imu_client.hpp"

#define SAMPLE_RATE_MAX 1000

// reconnect imud when disconnect
#define RETRY_MAX 30
#define RETEY_INTERVAL 1    // 1s

#define PACK_NUM_MAX 32

class ImuNode: public rclcpp::Node
{
public:
    ImuNode() : Node("imu_node")
    {
    }
    ~ImuNode();
    bool Init();
private:
    void KeepAlive();
    void PublishMsg();
    bool ImuClientInit();

    int _sample_rate;
    ImuClient _imu_client;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    bool _running = true;
    bool _connecting = false;
    std::shared_ptr<std::thread> _thread_keep_alive;
    std::shared_ptr<std::thread> _thread_publish_msg;
};

ImuNode::~ImuNode()
{
    RCLCPP_INFO(this->get_logger(), "stopping...");
    _running = false;
    if (_thread_publish_msg) {
        _thread_publish_msg->join();
    }
    _imu_client.DisconnectServer();
    if (_thread_keep_alive) {
        _thread_keep_alive->join();
    }
}

bool ImuNode::Init()
{
    _sample_rate = this->declare_parameter("sample_rate", 200);
    if (_sample_rate > SAMPLE_RATE_MAX || _sample_rate < 1) {
        RCLCPP_ERROR(this->get_logger(), "sample_rate must from 1 to 1000");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "sample_rate: %d", _sample_rate);

    if (!this->ImuClientInit()) return false;
    _connecting = true;

    _thread_keep_alive = std::make_shared<std::thread>(std::mem_fn(&ImuNode::KeepAlive), this);

    _publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 30);
    _thread_publish_msg = std::make_shared<std::thread>(std::mem_fn(&ImuNode::PublishMsg), this);

    return true;
}

bool ImuNode::ImuClientInit()
{
    if (!_imu_client.ConnectServer()) {
        RCLCPP_ERROR(this->get_logger(), "connect imud failed");
        return false;
    }

    if (!_imu_client.SendMsgConfigDataType(ACCEL_TYPE)
        || !_imu_client.SendMsgConfigRate(_sample_rate)) {
        RCLCPP_ERROR(this->get_logger(), "send CONFIG_RATE to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgConfigDataType(GYRO_TYPE)
        || !_imu_client.SendMsgConfigRate(_sample_rate)) {
        RCLCPP_ERROR(this->get_logger(), "send CONFIG_RATE to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgStart(ACCEL_TYPE)) {
        RCLCPP_ERROR(this->get_logger(), "send START accel to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgStart(GYRO_TYPE)) {
        RCLCPP_ERROR(this->get_logger(), "send START gyro to imud failed");
        return false;
    }

    if (!_imu_client.InitMmap()) {
        RCLCPP_ERROR(this->get_logger(), "init mmap failed");
        return false;
    }
    return true;
}

void ImuNode::KeepAlive()
{
    char buffer[64];
    int retry = 0;

    while(true) {
        int read_len = _imu_client.ReadMsg(buffer, 63);
        if (read_len > 0) continue;

        if (!_running) break;   // diconnect normally

        RCLCPP_ERROR(this->get_logger(), "can not connect imud");
        _connecting = false;

        while(retry++ < RETRY_MAX) {
            if (!_running) break;
            if (this->ImuClientInit()) {
                _connecting = true;
                retry = 0;
                RCLCPP_INFO(this->get_logger(), "reconnect succ");
                break;
            } else {
                RCLCPP_ERROR(this->get_logger(), "reconnect imud failed");
                RCLCPP_ERROR(this->get_logger(), "retry %d/%d after %d seconds...",
                    retry, RETRY_MAX, RETEY_INTERVAL);
                std::this_thread::sleep_for(std::chrono::seconds(RETEY_INTERVAL));
            }
        }

        if (retry >= RETRY_MAX) {
            RCLCPP_ERROR(this->get_logger(), "reconnect imud failed");
            break;
        }
    }
}

void ImuNode::PublishMsg()
{
    struct imu_pack_dsp imu_data[PACK_NUM_MAX];
    int32_t pack_num = 0;
    int i = 0;
    sensor_msgs::msg::Imu imu_msg;

    struct sysinfo sys_info;
    time_t now_ts = 0;
    time_t boot_ts = 0;
    sysinfo(&sys_info);
    time(&now_ts);
    boot_ts = now_ts - sys_info.uptime;

    while (_running) {
        pack_num = 0;
        if (_connecting && !_imu_client.GetImuData(imu_data, PACK_NUM_MAX, &pack_num)) {
            RCLCPP_WARN(this->get_logger(), "get imu data failed");
            continue;
        }
        for (i = 0; i < pack_num; i++) {
            imu_msg.header.stamp.nanosec  = imu_data[i].time_acc % 1000000000LL;
            imu_msg.header.stamp.sec      = imu_data[i].time_acc / 1000000000LL + boot_ts;
            imu_msg.linear_acceleration.x = imu_data[i].acceloration_x;
            imu_msg.linear_acceleration.y = imu_data[i].acceloration_y;
            imu_msg.linear_acceleration.z = imu_data[i].acceloration_z;
            imu_msg.angular_velocity.x    = imu_data[i].angular_velocity_x;
            imu_msg.angular_velocity.y    = imu_data[i].angular_velocity_y;
            imu_msg.angular_velocity.z    = imu_data[i].angular_velocity_z;
            _publisher->publish(imu_msg);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNode>();
    if (!node->Init()) {
        RCLCPP_ERROR(node->get_logger(), "init failed");
    } else {
        RCLCPP_INFO(node->get_logger(), "running...");
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
