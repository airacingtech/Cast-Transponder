#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "transponder_msgs/msg/transponder.hpp"

class TransponderDebug : public rclcpp::Node
{
  public:
    TransponderDebug()
    : Node("debug_node")
    {

        RCLCPP_INFO(this->get_logger(), "Transponder Debug Node");

        // Parameters
        this->declare_parameter("transponder_in", "/transponder/in");
        std::string param_transponderIn = this->get_parameter("transponder_in").as_string();

        // Publishers
        pub_Transponder_ = this->create_publisher<transponder_msgs::msg::Transponder>(param_transponderIn, 1);

        // Timers
        timer_debug_ = this->create_wall_timer(
            std::chrono::milliseconds(1 * 1000),
            std::bind(&TransponderDebug::callback_debug, this));
    }

  private:
    void callback_debug()
    {
        msg_.header.stamp = this->get_clock()->now();
        msg_.header.frame_id = "map";

        msg_.car_id = 8;
        msg_.state = transponder_msgs::msg::Transponder::STATE_NOMINAL;
        msg_.heartbeat = static_cast<uint8_t>(msg_.heartbeat + 1);
        msg_.lat_e7 += 1000000;   // +0.1 deg
        msg_.lon_e7 -= 1000000;   // -0.1 deg
        msg_.alt_dm = static_cast<int16_t>(msg_.alt_dm + 1);
        msg_.heading_cdeg = static_cast<int16_t>(msg_.heading_cdeg + 3);
        msg_.vel_cms = static_cast<int16_t>(msg_.vel_cms + 20);
        msg_.pass_state = transponder_msgs::msg::Transponder::PASS_STATE_REQUESTING;
        msg_.pass_sequence = static_cast<uint8_t>(msg_.pass_sequence + 1);
        msg_.target_car_id = 12;
        msg_.pass_zone_id = 1;
        msg_.yield_speed_cms = static_cast<uint16_t>(msg_.yield_speed_cms + 5);
        msg_.request_ttl_ms = 1000;

        pub_Transponder_->publish(msg_);
    }

    // Publishers / Subscribers / Timers
    rclcpp::TimerBase::SharedPtr timer_debug_;
    rclcpp::Publisher<transponder_msgs::msg::Transponder>::SharedPtr pub_Transponder_;

    // Variables
    transponder_msgs::msg::Transponder msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransponderDebug>());
  rclcpp::shutdown();
  return 0;
}
