
#pragma once

#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <functional>
#include <mutex>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <termio.h>
#include <stdio.h>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <glob.h>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "transponder_msgs/msg/transponder.hpp"

#include "iac_udp_struct.h"

class transponder2ros : public rclcpp::Node
{
public:
    transponder2ros();
    ~transponder2ros();

private:

    // Subscribers / Publishers / Timers
    rclcpp::Subscription<transponder_msgs::msg::Transponder>::SharedPtr sub_Transponder_;

    rclcpp::Publisher<transponder_msgs::msg::Transponder>::SharedPtr pub_Transponder_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_Version_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_LinkStatus_;

    rclcpp::TimerBase::SharedPtr timer_1Hz_, timer_10s_;
    rclcpp::TimerBase::SharedPtr timer_readSerial_;

    // Threads
    std::thread receive_data_thread_;

    // Variables
    sockaddr_in send_addr_;

    char buffer_[1024];
    struct sockaddr_in client_addr_;
    socklen_t addr_len_ = sizeof(client_addr_);
    int read_addr_; //socket_fd;

    rclcpp::Time t_last_packet_ = this->get_clock()->now();
    double t_Udp_maxAge_;   // Max age of UDP packets to accept
    double t_Udp_timeout_;  // Timeout before warning user of no packets
    bool has_timeout_ = false;

    int sockfd_ = socket(AF_INET,SOCK_DGRAM,0);
    int m_serialPort_ = 0;

    std::mutex lock_;

    bool debug_RawData_ = 0;

    // Functions
    void init_udp();
    void init_ros();
    void init_serial();

    void push_udp(StructIacTransponder data);
    void read_udpData();

    bool open_serial(int &p_serialPort, std::string device_path);
    void read_serialData();
    bool parseChar(unsigned char x);
    uint8_t calc_crc8(const char* data, size_t len);

    void callback_Transponder(const transponder_msgs::msg::Transponder::SharedPtr msg);
    void publish_Transponder(TransponderUdpPacket transponder);
    void publish_link_status();

    void callback_1Hz();
    void callback_10s();
  
};
