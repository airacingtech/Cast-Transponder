
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
    // Sender of the datagram recvfrom just returned. Must be a sockaddr_in, not an int: recvfrom
    // writes addr_len_ bytes here, so a 4-byte member let the kernel scribble 12 bytes over
    // whatever followed it -- which was t_last_packet_, whose clock type it zeroed.
    struct sockaddr_in read_addr_{};
    socklen_t addr_len_ = sizeof(read_addr_);

    // Two independent silences, deliberately not one flag. A heartbeat proves the unit is
    // reachable but says nothing about the radio, so "the unit is gone" and "no car is
    // broadcasting" are different faults with different deadlines and different audiences --
    // link_lost_ gates the published link status, no_car_data_ only drives an operator warning.
    //
    // Everything from here to notified_timeout_silence_ is written by the UDP listener thread and
    // by the executor, so it is guarded by lock_. The timestamps are the reason a mutex is needed
    // rather than atomics: rclcpp::Time is 16 bytes and a torn read yields a nonsense age.
    rclcpp::Time t_last_packet_ = this->get_clock()->now();      // any packet, heartbeat included
    rclcpp::Time t_last_car_packet_ = this->get_clock()->now();  // packets carrying vehicle data
    double t_Udp_maxAge_;     // Max age of UDP packets to accept
    double t_Udp_timeout_;    // Timeout before warning user of no data from other cars
    double t_Link_timeout_;   // Silence from the unit itself before the link counts as lost
    // Not derived from t_last_packet_, which is seeded at construction and so cannot tell "nothing
    // has ever arrived" from "a packet arrived the instant we started". A consumer gates the rival
    // feed on the link status, and claiming a working link before the first packet is the one
    // answer that is never safe, so the never-heard-anything case gets its own flag.
    bool packet_ever_received_ = false;
    bool link_lost_ = true;
    bool no_car_data_ = false;
    bool notified_timeout_silence_ = false;

    int sockfd_ = socket(AF_INET,SOCK_DGRAM,0);
    int m_serialPort_ = 0;

    std::mutex lock_;

    bool debug_RawData_ = 0;

    // Functions
    void init_udp();
    void init_ros();
    void init_serial();
    void start_udp_listener();

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
