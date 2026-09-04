#include "transponder2ros/transponder2ros.hpp"

void transponder2ros::init_ros()
{
    // Parameters
    this->declare_parameter("transponder_in", "/transponder/in");
    this->declare_parameter("transponder_out", "/transponder/out");
    this->declare_parameter("version_out", "/transponder/version");
    this->declare_parameter("link_status_out", "/transponder/link_status");

    std::string param_transponderIn = this->get_parameter("transponder_in").as_string();
    std::string param_transponderOut = this->get_parameter("transponder_out").as_string();
    std::string param_versionOut = this->get_parameter("version_out").as_string();
    std::string param_linkStatusOut = this->get_parameter("link_status_out").as_string();

    // Publishers
    pub_Transponder_ = this->create_publisher<transponder_msgs::msg::Transponder>(param_transponderIn, 1);
    pub_Version_ = this->create_publisher<std_msgs::msg::String>(param_versionOut, 10);
    pub_LinkStatus_ = this->create_publisher<std_msgs::msg::Bool>(param_linkStatusOut, 10);

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

    // Update our last received time and reconnect status for every packet, including heartbeats.
    // A heartbeat proves the local Ethernet link, the transponder unit, and its firmware loop are
    // all alive even when nothing has been heard over the XBee radio -- that's what distinguishes
    // "no vehicle detected" from "we can't detect the transponder at all."
    t_last_packet_ = this->get_clock()->now();
    if (has_timeout_)
    {
        RCLCPP_INFO(this->get_logger(), "Transponder connected");
        has_timeout_ = false;
        publish_link_status();
    }

    // Heartbeat packets carry no vehicle data -- they exist only to prove the link is alive.
    if (transponder.data.car_id == HEARTBEAT_CAR_ID)
    {
        return;
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
    msg.lat = transponder.data.lat/1e7;
    msg.lon = transponder.data.lon/1e7;
    msg.alt = transponder.data.alt/1e3;
    msg.heading = transponder.data.heading/1e2;
    msg.vel = transponder.data.vel/1e2;
    msg.state = transponder.data.state;

    pub_Transponder_->publish(msg);

    // Debugging
    if (0)
    {
        RCLCPP_INFO(this->get_logger(), "lat: %8.3d, lon: %8.3d",
            transponder.data.lat, transponder.data.lon
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
    transponder.car_id = msg->car_id;                       // Car ID [ - ]
    transponder.lat = msg->lat*1e7;                         // Vehicle longitude [ dd.dd x 10^7 ]
    transponder.lon = msg->lon*1e7;                         // Vehicle latitude [ dd.dd x 10^7 ]
    transponder.alt = msg->alt*1e3;                         // Vehicle altitude [ mm ]
    transponder.heading = msg->heading*1e2;                 // Vehicle GPS heading [ cdeg ]
    transponder.vel = (msg->vel > 0) ? msg->vel*1e2 : 0;    // Vehicle speed, force positive [ cm/s ]
    transponder.state = msg->state;                         // Vehicle state [ - ]
  
    // Push data
    push_udp(transponder);

    // Done
    return;
}

void transponder2ros::publish_link_status()
{
    std_msgs::msg::Bool msg;
    msg.data = !has_timeout_;
    pub_LinkStatus_->publish(msg);
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

    publish_link_status();

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
