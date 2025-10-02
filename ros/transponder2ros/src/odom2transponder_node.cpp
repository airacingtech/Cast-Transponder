
// Example node of publishing a transponder message from an odom message

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "transponder_msgs/msg/transponder.hpp"

class Odom2Transponder : public rclcpp::Node
{
public:
    Odom2Transponder()
        : Node("odom2transponder")
    {

        RCLCPP_INFO(this->get_logger(), "Odomentry-to-Transponder Node");

        // Parameters
        this->declare_parameter("odometry_in", "/state/odom");
        this->declare_parameter("carmode_in", "/trajectory/mode/requested");
        this->declare_parameter("transponder_out", "/transponder/out");
        std::string param_odometryIn = this->get_parameter("odometry_in").as_string();
        std::string param_carModeIn = this->get_parameter("carmode_in").as_string();
        std::string param_transponderOut = this->get_parameter("transponder_out").as_string();

        this->declare_parameter<double>("lat0", 0.0);
        this->declare_parameter<double>("lon0", 0.0);
        this->declare_parameter<double>("alt0", 0.0);
        lla0_.latitude = this->get_parameter("lat0").as_double();
        lla0_.longitude = this->get_parameter("lon0").as_double();
        lla0_.altitude = this->get_parameter("alt0").as_double();

        this->declare_parameter<int>("car", 1);  // Car number
        car_id_ = this->get_parameter("car").as_int();

        // Publishers
        pub_Transponder_ = this->create_publisher<transponder_msgs::msg::Transponder>(param_transponderOut, 1);

        // Subscribers
        sub_Odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            param_odometryIn,
            1,
            std::bind(
                &Odom2Transponder::callback_Odometry,
                this, std::placeholders::_1)
        );

        /*
        // For those that want to fill in the 'state' field, here's an example of how to do it
        // Each different team will have different modes, so this is just an example for CAST Racer
        sub_CarMode_ = this->create_subscription<iac_msgs::msg::CarMode>(
            param_carModeIn,
            1,
            std::bind(
                &Odom2Transponder::callback_CarMode,
                this, std::placeholders::_1)
        );
        */

        // Timers
        timer_pushTransponder_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Kept at a reasonably rate to not flood the network
            std::bind(&Odom2Transponder::callback_pushTransponder, this));
    }

private:
    void callback_pushTransponder()
    {
        // Check timestamp is reasonable
        rclcpp::Duration t_late_by = this->get_clock()->now() - odom_.header.stamp;
        if (abs(t_late_by.seconds()) > 1.0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5UL * 1000 * 1000, "Odom late by %.1f",t_late_by.seconds());
            // RCLCPP_WARN(this->get_logger(), "Odom late by %.1f",t_late_by.seconds());
            return;
        }

        // Convert enu to lla
        geographic_msgs::msg::GeoPoint lla = enu_to_lla_geodetic(odom_.pose.pose.position, lla0_);

        // Calculate yaw
        geometry_msgs::msg::Quaternion q = odom_.pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        double yaw_deg = (M_PI_2-yaw)*180.0/M_PI;
        while (yaw_deg < 0.0) yaw_deg += 360.0;

        // Push message
        transponder_msgs::msg::Transponder msg;

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        msg.car_id = car_id_;
        msg.state = car_mode_;
        msg.heartbeat = heartbeat_++;

        const auto lat_e7 = std::llround(lla.latitude * 1e7);
        const auto lon_e7 = std::llround(lla.longitude * 1e7);
        const auto alt_dm = std::llround(lla.altitude * 10.0);

        msg.lat_e7 = static_cast<int32_t>(std::clamp<long long>(
            lat_e7,
            static_cast<long long>(std::numeric_limits<int32_t>::min()),
            static_cast<long long>(std::numeric_limits<int32_t>::max())));

        msg.lon_e7 = static_cast<int32_t>(std::clamp<long long>(
            lon_e7,
            static_cast<long long>(std::numeric_limits<int32_t>::min()),
            static_cast<long long>(std::numeric_limits<int32_t>::max())));

        msg.alt_dm = static_cast<int16_t>(std::clamp<long long>(
            alt_dm,
            static_cast<long long>(std::numeric_limits<int16_t>::min()),
            static_cast<long long>(std::numeric_limits<int16_t>::max())));

        double heading_deg = yaw_deg;
        if (heading_deg > 180.0)
        {
            heading_deg -= 360.0;
        }

        const auto heading_cdeg = std::llround(heading_deg * 100.0);
        msg.heading_cdeg = static_cast<int16_t>(std::clamp<long long>(
            heading_cdeg,
            static_cast<long long>(std::numeric_limits<int16_t>::min()),
            static_cast<long long>(std::numeric_limits<int16_t>::max())));

        auto vel_cms = std::llround(odom_.twist.twist.linear.x * 100.0);
        if (vel_cms < 0)
        {
            vel_cms = 0;
        }
        msg.vel_cms = static_cast<int16_t>(std::clamp<long long>(
            vel_cms,
            static_cast<long long>(std::numeric_limits<int16_t>::min()),
            static_cast<long long>(std::numeric_limits<int16_t>::max())));

        msg.pass_state = transponder_msgs::msg::Transponder::PASS_STATE_IDLE;
        msg.pass_sequence = 0;
        msg.target_car_id = 0;
        msg.pass_zone_id = 0;
        msg.yield_speed_cms = 0;
        msg.request_ttl_ms = 0;

        pub_Transponder_->publish(msg);

    }

    void callback_Odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Save the latest odom data
        odom_ = *msg;

        // Done
        return;
    }

    /*
    // For those that want to fill in the 'state' field, here's an example of how to do it
    // Each different team will have different modes, so this is just an example for CAST Racer

    void callback_CarMode(const iac_msgs::msg::CarMode::SharedPtr msg)
    {
        // Save the latest odom data
        switch (msg->mode)
        {
            case (iac_msgs::msg::CarMode::TERMINATE) :
            case (iac_msgs::msg::CarMode::EMERGENCY_STOP) :
                car_mode_ = transponder_msgs::msg::Transponder::STATE_EMERGENCY_STOP;
                break;
            case (iac_msgs::msg::CarMode::CONTROLLED_STOP) :
                car_mode_ = transponder_msgs::msg::Transponder::STATE_CONTROLLED_STOP;
                break;
            case (iac_msgs::msg::CarMode::ENGINE_IDLE) :
            case (iac_msgs::msg::CarMode::PIT) :
            case (iac_msgs::msg::CarMode::ADAPTIVE_CRUISE) :
            case (iac_msgs::msg::CarMode::OVERTAKE_ALLOWED) :
            case (iac_msgs::msg::CarMode::DEFENDER) :
            case (iac_msgs::msg::CarMode::RACE) :
                car_mode_ = transponder_msgs::msg::Transponder::STATE_NOMINAL;
                break;
            case (iac_msgs::msg::CarMode::UNKNOWN) :
            default:
                car_mode_ = transponder_msgs::msg::Transponder::STATE_UNKNOWN;
                break;
        }

        // Done
        return;
    }
    */

    geographic_msgs::msg::GeoPoint enu_to_lla_geodetic(
        const geometry_msgs::msg::Point enu,
        const geographic_msgs::msg::GeoPoint lla0)
    {
        static GeographicLib::LocalCartesian local_cartesian(
            lla0.latitude, lla0.longitude, lla0.altitude,
            GeographicLib::Geocentric::WGS84()
        );

        // Convert local Cartesian coordinates to geodetic coordinates
        geographic_msgs::msg::GeoPoint out;
        local_cartesian.Reverse(
            enu.x, enu.y, enu.z,
            out.latitude, out.longitude, out.altitude
        );

        return out;
    }

    // Publishers / Subscribers / Timers
    rclcpp::Publisher<transponder_msgs::msg::Transponder>::SharedPtr pub_Transponder_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_Odometry_;
    // rclcpp::Subscription<iac_msgs::msg::CarMode>::SharedPtr sub_CarMode_;  // Needed to fill in the 'state' field
    rclcpp::TimerBase::SharedPtr timer_pushTransponder_;

    // Variables
    uint8_t car_id_;
    nav_msgs::msg::Odometry odom_;
    geographic_msgs::msg::GeoPoint lla0_;
    uint8_t car_mode_ = transponder_msgs::msg::Transponder::STATE_UNKNOWN;
    uint8_t heartbeat_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odom2Transponder>());
    rclcpp::shutdown();
    return 0;
}
