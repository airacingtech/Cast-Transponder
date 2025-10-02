#include "rclcpp/rclcpp.hpp"
#include "transponder_msgs/msg/transponder.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class Transponder2NavSatFixNode : public rclcpp::Node
{
public:
    Transponder2NavSatFixNode() : Node("vehicle_to_gps_node")
    {

        // Parameters
        this->declare_parameter("transponder_in", "in");
        this->declare_parameter("lla_out", "lla");
        std::string param_transponderIn = this->get_parameter("transponder_in").as_string();
        std::string param_llaOut = this->get_parameter("lla_out").as_string();

        // Publishers
        pub_NavSatFix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            param_llaOut,
            rclcpp::SensorDataQoS()
        );

        // Subscribers
        sub_Transponder_ = this->create_subscription<transponder_msgs::msg::Transponder>(
            param_transponderIn,
            1,
            std::bind(
                &Transponder2NavSatFixNode::callback_Transponder,
                this, std::placeholders::_1)
        );

    }

private:
    void callback_Transponder(const transponder_msgs::msg::Transponder::SharedPtr msg)
    {
        sensor_msgs::msg::NavSatFix gps_msg;
        gps_msg.header = msg->header;
        gps_msg.latitude = static_cast<double>(msg->lat_e7) / 1e7;
        gps_msg.longitude = static_cast<double>(msg->lon_e7) / 1e7;
        gps_msg.altitude = static_cast<double>(msg->alt_dm) / 10.0;
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; // Assume valid fix
        gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        
        pub_NavSatFix_->publish(gps_msg);
    }

    rclcpp::Subscription<transponder_msgs::msg::Transponder>::SharedPtr sub_Transponder_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_NavSatFix_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transponder2NavSatFixNode>());
    rclcpp::shutdown();
    return 0;
}
