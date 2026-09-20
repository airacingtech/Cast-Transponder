#include "transponder2ros/transponder2ros.hpp"

transponder2ros::transponder2ros()
    : Node("transponder_node")
{

    RCLCPP_INFO(this->get_logger(), "Transponder Node");

    // Init the UDP connection 
    init_udp();

    // Init a Serial connection
    init_serial();

    // Init ROS
    init_ros();

    // Last: the listener publishes, so every publisher it touches has to exist first.
    start_udp_listener();

    // All done
    return;
}

transponder2ros::~transponder2ros()
{
    if (receive_data_thread_.joinable())
    {
        receive_data_thread_.join();
    }
    // The socket, not read_addr_ -- that holds the last sender's address and closing it amounted
    // to closing whatever descriptor number the peer's IP bytes happened to spell.
    if (sockfd_ != -1)
    {
        close(sockfd_);
    }
}