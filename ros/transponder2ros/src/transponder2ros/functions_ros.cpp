#include "transponder2ros/transponder2ros.hpp"

void transponder2ros::init_ros()
{
    // Parameters
    this->declare_parameter("transponder_in", "/transponder/in");
    this->declare_parameter("transponder_out", "/transponder/out");
    this->declare_parameter("coordination_in", "/transponder/coordination_in");
    this->declare_parameter("coordination_out", "/transponder/coordination_out");
    this->declare_parameter("version_out", "/transponder/version");

    std::string param_transponderIn  = this->get_parameter("transponder_in").as_string();
    std::string param_transponderOut = this->get_parameter("transponder_out").as_string();
    std::string param_coordinationIn  = this->get_parameter("coordination_in").as_string();
    std::string param_coordinationOut = this->get_parameter("coordination_out").as_string();
    std::string param_versionOut     = this->get_parameter("version_out").as_string();

    // Publishers
    pub_Transponder_  = this->create_publisher<transponder_msgs::msg::Transponder>(param_transponderIn, 1);
    pub_Coordination_ = this->create_publisher<transponder_msgs::msg::AVLTCoordination>(param_coordinationIn, 1);
    pub_Version_      = this->create_publisher<std_msgs::msg::String>(param_versionOut, 10);

    // Subscribers
    sub_Transponder_ = this->create_subscription<transponder_msgs::msg::Transponder>(
        param_transponderOut,
        1,
        std::bind(&transponder2ros::callback_Transponder, this, std::placeholders::_1));

    sub_Coordination_ = this->create_subscription<transponder_msgs::msg::AVLTCoordination>(
        param_coordinationOut,
        1,
        std::bind(&transponder2ros::callback_Coordination, this, std::placeholders::_1));

    // Timers
    timer_1Hz_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&transponder2ros::callback_1Hz, this));

    timer_10s_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&transponder2ros::callback_10s, this));

}

void transponder2ros::publish_Transponder(PositionUdpPacket transponder)
{

    // Reject if the versions don't match
    if (transponder.data.version != TRANSPONDER_UDP_STRUCT_VERISON)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Version mismatch | car: %d, ours: 0x%02X, theirs: 0x%02X",
            transponder.data.car_id, TRANSPONDER_UDP_STRUCT_VERISON, transponder.data.version
        );
        return;
    }

    // Reject if message is too old
    rclcpp::Time t_data(transponder.data.sec, transponder.data.nanosec, this->get_clock()->get_clock_type());
    rclcpp::Duration msg_tDiff = this->get_clock()->now() - t_data;

    if (std::abs(msg_tDiff.seconds()) > t_Udp_maxAge_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "Transponder UDP packet late by %.3f s", msg_tDiff.seconds());
        return;
    }

    // Publish the data
    transponder_msgs::msg::Transponder msg;

    msg.header.stamp.sec     = transponder.data.sec;
    msg.header.stamp.nanosec = transponder.data.nanosec;
    msg.header.frame_id      = "map";

    msg.car_id    = transponder.data.car_id;
    msg.heartbeat = transponder.data.heartbeat;
    msg.lat       = transponder.data.lat / 1e7;
    msg.lon       = transponder.data.lon / 1e7;
    msg.alt       = transponder.data.alt / 1e3;
    msg.heading   = transponder.data.heading / 1e2;
    msg.vel       = transponder.data.vel / 1e2;
    msg.state     = transponder.data.state;

    pub_Transponder_->publish(msg);

    // Update our last received time
    t_last_packet_ = this->get_clock()->now();

    // Update user if reconnected
    if (has_timeout_)
    {
        RCLCPP_INFO(this->get_logger(), "Transponder connected");
        has_timeout_ = false;
    }

    // Done
    return;

}

void transponder2ros::callback_Transponder(const transponder_msgs::msg::Transponder::SharedPtr msg)
{
    static uint8_t heartbeat_counter = 0;

    StructAVLTPosition transponder;

    transponder.version    = TRANSPONDER_UDP_STRUCT_VERISON;
    transponder.packet_type = PACKET_TYPE_POSITION;
    transponder.sec        = msg->header.stamp.sec;
    transponder.nanosec    = msg->header.stamp.nanosec;
    transponder.car_id     = msg->car_id;
    transponder.heartbeat  = heartbeat_counter++;
    transponder.lat        = msg->lat * 1e7;
    transponder.lon        = msg->lon * 1e7;
    transponder.alt        = msg->alt * 1e3;
    transponder.heading    = msg->heading * 1e2;
    transponder.vel        = (msg->vel > 0) ? msg->vel * 1e2 : 0;
    transponder.state      = msg->state;

    // Push data
    push_udp(transponder);

    // Done
    return;
}

void transponder2ros::publish_Coordination(CoordinationUdpPacket coord)
{

    // Reject if the versions don't match
    if (coord.data.version != TRANSPONDER_UDP_STRUCT_VERISON)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Coordination version mismatch | car: %d, ours: 0x%02X, theirs: 0x%02X",
            coord.data.car_id, TRANSPONDER_UDP_STRUCT_VERISON, coord.data.version
        );
        return;
    }

    // Reject if message is too old
    rclcpp::Time t_data(coord.data.sec, coord.data.nanosec, this->get_clock()->get_clock_type());
    rclcpp::Duration msg_tDiff = this->get_clock()->now() - t_data;

    if (std::abs(msg_tDiff.seconds()) > t_Udp_maxAge_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "Coordination UDP packet late by %.3f s", msg_tDiff.seconds());
        return;
    }

    // Publish the data
    transponder_msgs::msg::AVLTCoordination msg;

    msg.header.stamp.sec     = coord.data.sec;
    msg.header.stamp.nanosec = coord.data.nanosec;
    msg.header.frame_id      = "map";

    msg.car_id          = coord.data.car_id;
    msg.pass_state      = coord.data.pass_state;
    msg.pass_sequence   = coord.data.pass_sequence;
    msg.target_car_id   = coord.data.target_car_id;
    msg.pass_zone_id    = coord.data.pass_zone_id;
    msg.yield_speed     = coord.data.yield_speed / 1e2f;
    msg.request_ttl_ms  = coord.data.request_ttl_ms;

    pub_Coordination_->publish(msg);

    // Update our last received time
    t_last_packet_ = this->get_clock()->now();

    // Update user if reconnected
    if (has_timeout_)
    {
        RCLCPP_INFO(this->get_logger(), "Transponder connected");
        has_timeout_ = false;
    }

    // Done
    return;
}

void transponder2ros::callback_Coordination(const transponder_msgs::msg::AVLTCoordination::SharedPtr msg)
{
    StructAVLTCoordination coord;

    coord.version        = TRANSPONDER_UDP_STRUCT_VERISON;
    coord.packet_type    = PACKET_TYPE_COORDINATION;
    coord.sec            = msg->header.stamp.sec;
    coord.nanosec        = msg->header.stamp.nanosec;
    coord.car_id         = msg->car_id;
    coord.pass_state     = msg->pass_state;
    coord.pass_sequence  = msg->pass_sequence;
    coord.target_car_id  = msg->target_car_id;
    coord.pass_zone_id   = msg->pass_zone_id;
    coord.yield_speed    = (msg->yield_speed > 0) ? (uint16_t)(msg->yield_speed * 1e2f) : 0;
    coord.request_ttl_ms = msg->request_ttl_ms;

    // Push data
    push_udp_coordination(coord);

    // Done
    return;
}

void transponder2ros::callback_1Hz()
{

    rclcpp::Duration t_since_last_msg = this->get_clock()->now() - t_last_packet_;
    static bool notified_timeout_silence = false;

    if (t_since_last_msg.seconds() > t_Udp_timeout_)
    {
        has_timeout_ = true;
    }
    else
    {
        notified_timeout_silence = false;
    }

    // Print message for up to 30 s
    if (has_timeout_ && t_since_last_msg.seconds() < 30.0)
    {
        // Timeout
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5UL * 1000 * 1000,
            "No transponder data from other cars in %.1f s", t_since_last_msg.seconds()
        );
    }
    else if (t_since_last_msg.seconds() > 30.0 && notified_timeout_silence == 0)
    {
        RCLCPP_INFO(this->get_logger(), ">> Silencing transponder timeout message");
        notified_timeout_silence = true;
    }

    // All done
    return;
}

void transponder2ros::callback_10s()
{

    // Publish a version string for debugging
    std::string software_version = GIT_INFO;
    std_msgs::msg::String msg;
    msg.data = "transponder (UDP): " + software_version;
    pub_Version_->publish(msg);

    // All done
    return;
}
