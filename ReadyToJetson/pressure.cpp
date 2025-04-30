#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

extern "C" {
    #include </home/elfikry/c_library_v2/common/mavlink.h>  // Ganti path ini jika berbeda
}

#define BUFFER_LENGTH 2048
#define UDP_PORT 14551

class MavlinkPressurePublisher : public rclcpp::Node {
public:
    MavlinkPressurePublisher()
        : Node("mavlink_pressure_publisher") {
        
        publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("pressure", 10);
        open_udp_socket();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&MavlinkPressurePublisher::poll_udp, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int sockfd_;
    struct sockaddr_in remote_addr_;
    socklen_t addr_len_ = sizeof(remote_addr_);

    void open_udp_socket() {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuat socket UDP");
            rclcpp::shutdown();
        }

        struct sockaddr_in local_addr{};
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(UDP_PORT);

        if (bind(sockfd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Gagal bind ke port UDP");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Mendengarkan MAVLink di UDP port %d", UDP_PORT);
    }

    void poll_udp() {
        uint8_t buffer[BUFFER_LENGTH];
        ssize_t nbytes = recvfrom(sockfd_, buffer, BUFFER_LENGTH, MSG_DONTWAIT,
                                  (struct sockaddr *)&remote_addr_, &addr_len_);

        if (nbytes <= 0) return;

        mavlink_message_t msg;
        mavlink_status_t status;

        for (ssize_t i = 0; i < nbytes; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_SCALED_PRESSURE2) {
                    mavlink_scaled_pressure2_t pressure;
                    mavlink_msg_scaled_pressure2_decode(&msg, &pressure);

                    auto pressure_msg = sensor_msgs::msg::FluidPressure();
                    pressure_msg.header.stamp = this->now();
                    pressure_msg.header.frame_id = "base_link";  // Bisa diganti sesuai setup

                    pressure_msg.fluid_pressure = pressure.press_abs;  // hPa to Pa
                    pressure_msg.variance = 0.0;  // Tidak tersedia dari MAVLink

                    publisher_->publish(pressure_msg);

                    RCLCPP_INFO(this->get_logger(), "tekanan Terbaca: %.2f hPa", pressure_msg.fluid_pressure);
                }
            }
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavlinkPressurePublisher>());
    rclcpp::shutdown();
    return 0;
}
