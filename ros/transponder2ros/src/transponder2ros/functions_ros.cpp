#include "transponder2ros/transponder2ros.hpp"

void transponder2ros::init_ros()
{
    // Parameters
    this->declare_parameter("transponder_in", "/transponder/in");
    this->declare_parameter("transponder_out", "/transponder/out");
    this->declare_parameter("version_out", "/transponder/version");

    std::string param_transponderIn = this->get_parameter("transponder_in").as_string();
    std::string param_transponderOut = this->get_parameter("transponder_out").as_string();
    std::string param_versionOut = this->get_parameter("version_out").as_string();

    // Publishers
    pub_Transponder_ = this->create_publisher<transponder_msgs::msg::Transponder>(param_transponderIn, 1);
    pub_Version_ = this->create_publisher<std_msgs::msg::String>(param_versionOut, 10);

    // Subscribers
    sub_Transponder_ = this->create_subscription<transponder_msgs::msg::Transponder>(
        param_transponderOut,
        1,
        std::bind(&transponder2ros::callback_Transponder, this, std::placeholders::_1));

    // Timers
    timer_1Hz_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&transponder2ros::callback_1Hz, this));

    timer_10s_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&transponder2ros::callback_10s, this));

}

void transponder2ros::publish_Transponder(TransponderUdpPacket transponder)
{

    // Reject if the versions don't match
    if (transponder.data.version != TRANSPONDER_UDP_STRUCT_VERISON)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Version mismatch | car: %d, ours: 0x%02X, theirs: 0x%02X",
            transponder.data.car_id, TRANSPONDER_UDP_STRUCT_VERISON, transponder.data.version
        );
    }

    // Reject if message is too old
    rclcpp::Time t_data(transponder.data.sec, transponder.data.nanosec, this->get_clock()->get_clock_type());
    rclcpp::Duration msg_tDiff = this->get_clock()->now() - t_data;

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Latency: %.3f s", msg_tDiff);

    if (std::abs(msg_tDiff.seconds()) > t_Udp_maxAge_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "Transponder UDP packet late by %.3f s", msg_tDiff.seconds());
        return;
    }

    // Publish the data
    transponder_msgs::msg::Transponder msg;

    msg.header.stamp.sec = transponder.data.sec;
    msg.header.stamp.nanosec = transponder.data.nanosec;
    msg.header.frame_id = "map";

    msg.car_id = transponder.data.car_id;
    msg.state = transponder.data.state;
    msg.heartbeat = transponder.data.heartbeat;
    msg.lat_e7 = transponder.data.lat_e7;
    msg.lon_e7 = transponder.data.lon_e7;
    msg.alt_dm = transponder.data.alt_dm;
    msg.heading_cdeg = transponder.data.heading_cdeg;
    msg.vel_cms = transponder.data.vel_cms;
    msg.pass_state = transponder.data.pass_state;
    msg.pass_sequence = transponder.data.pass_sequence;
    msg.target_car_id = transponder.data.target_car_id;
    msg.pass_zone_id = transponder.data.pass_zone_id;
    msg.yield_speed_cms = transponder.data.yield_speed_cms;
    msg.request_ttl_ms = transponder.data.request_ttl_ms;

    pub_Transponder_->publish(msg);

    // Update our last received time
    t_last_packet_ = this->get_clock()->now();

    // Update user if reconnected
    if (has_timeout_)
    {
        RCLCPP_INFO(this->get_logger(), "Transponder connected");
        has_timeout_ = false;
    }

    // Debugging
    if (0)
    {
        RCLCPP_INFO(this->get_logger(), "lat_e7: %d, lon_e7: %d",
            transponder.data.lat_e7, transponder.data.lon_e7
        );
    }

    // Done
    return;

}

void transponder2ros::callback_Transponder(const transponder_msgs::msg::Transponder::SharedPtr msg)
{
    // Need to send Transponder data from here
    StructIacTransponder transponder;

    transponder.version = TRANSPONDER_UDP_STRUCT_VERISON;   // Struct version
    transponder.sec = msg->header.stamp.sec;                // UTC time [ s ]
    transponder.nanosec = msg->header.stamp.nanosec;        // UTC time nanoseconds [ ns ]
    transponder.car_id = msg->car_id;                       // Vehicle ID [ - ]
    transponder.state = msg->state;                         // Vehicle state [ - ]
    transponder.heartbeat = msg->heartbeat;                 // Heartbeat counter [ - ]
    transponder.lat_e7 = msg->lat_e7;                       // Latitude * 1e7 [ signed deg * 1e7 ]
    transponder.lon_e7 = msg->lon_e7;                       // Longitude * 1e7 [ signed deg * 1e7 ]
    transponder.alt_dm = msg->alt_dm;                       // Altitude [ dm ]
    transponder.heading_cdeg = msg->heading_cdeg;           // Heading [ cdeg ]
    transponder.vel_cms = (msg->vel_cms > 0) ? msg->vel_cms : 0; // Longitudinal speed [ cm/s ]
    transponder.pass_state = msg->pass_state;               // Pass FSM state [ enum ]
    transponder.pass_sequence = msg->pass_sequence;         // Pass handshake monotonic counter
    transponder.target_car_id = msg->target_car_id;         // Defender car ID [ - ]
    transponder.pass_zone_id = msg->pass_zone_id;           // Pass zone identifier
    transponder.yield_speed_cms = msg->yield_speed_cms;     // Defender follow speed [ cm/s ]
    transponder.request_ttl_ms = msg->request_ttl_ms;       // Request TTL [ ms ]

    // Push data
    push_udp(transponder);

    // Done
    return;
}

void transponder2ros::callback_1Hz()
{

    rclcpp::Duration t_since_last_msg = this->get_clock()->now() - t_last_packet_;
    static bool notified_timeout_silence = false;

    // Check timeouts (only used for printing in this case)
    // RCLCPP_INFO(this->get_logger(), "t_since_last_msg: %6.3f",t_since_last_msg.seconds());

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

    // RCLCPP_WARN(this->get_logger(), "10 s callback");

    // Publish a version string for debugging
    std::string software_version = GIT_INFO;
    std_msgs::msg::String msg;
    msg.data = "transponder (UDP): " + software_version;
    pub_Version_->publish(msg);

    // All done
    return;
}
