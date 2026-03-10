#include "transponder2ros/transponder2ros.hpp"

void transponder2ros::init_serial()
{

    // Get the param
    this->declare_parameter("device_id", "/dev/ttyUSB0");
    std::string param_dev = this->get_parameter("device_id").as_string();

    if (param_dev == "")
    {
        RCLCPP_INFO(this->get_logger(), "\tNo direct XBee link");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "\tTransponder XBee link at %s",param_dev.c_str());

    // Attempt to connect
    if (open_serial(m_serialPort_, param_dev))
    {
        // Add read data callbacks
        timer_readSerial_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&transponder2ros::read_serialData, this)
        );
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Device %s not plugged in, not using USB device", param_dev.c_str());
    }

    // Done
    return;
}

bool transponder2ros::open_serial(int &p_serialPort, std::string device_path)
{
    close(p_serialPort);
    usleep(5000);

    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));
    int glob_ret = glob(device_path.c_str(), GLOB_TILDE, NULL, &glob_result);

    if (glob_ret != 0)
    {
        globfree(&glob_result);
        RCLCPP_ERROR(this->get_logger(), "Failed to find device. Check connection and power.");
        return false;
    }

    device_path = std::string(glob_result.gl_pathv[0]);
    globfree(&glob_result);

    // Open serial port
    RCLCPP_INFO(this->get_logger(), "Opening device on %s", device_path.c_str());

    p_serialPort = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (p_serialPort == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open device  %s: %i, %s\n", device_path.c_str(), errno, strerror(errno));
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Opened device %s", device_path.c_str());

    // Set serial attributes
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(p_serialPort, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
        rclcpp::shutdown();
        return false;
    }

    /* Set Baud Rate */
    cfsetospeed(&tty, (speed_t)B57600);
    cfsetispeed(&tty, (speed_t)B57600);

    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB; // Make n81
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;       // no flow control
    tty.c_cc[VMIN] = 1;            // read doesn't block
    tty.c_cc[VTIME] = 5;           // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush(p_serialPort, TCIFLUSH);
    if (tcsetattr(p_serialPort, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s.", errno, strerror(errno));
        return false;
    }
    return true;
}

void transponder2ros::read_serialData()
{

    // Check if serial port open
    if (m_serialPort_ < 0)
    {
        return;
    }

    // read stuff
    int n_read = 0;
    unsigned char buf[6400] = {0};
    try
    {
        // read the buffer
        n_read = read(m_serialPort_, &buf, 6400);
    }
    catch (const std::exception &e)
    {
        // catch the error
        RCLCPP_ERROR(this->get_logger(), "Error %s trying to read device, probably unplugged", e.what());
    }

    // Parse data through state machine
    if (n_read > 0)
    {
        if (debug_RawData_)
        {
            if (1)
            {
                // Hex
                std::stringstream ss;
                for (int ii = 0; ii < n_read; ii++)
                {
                    ss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(buf[ii]) << " ";
                }
                RCLCPP_INFO(this->get_logger(), "Read %2d bytes | %s", n_read, ss.str().c_str());
            }
            if (0)
            {
                // Chars
                RCLCPP_INFO(this->get_logger(), "Read %2d bytes | %s", n_read, buf);
            }
        }

        for (int ii = 0; ii < n_read; ii++)
        {
            parseChar(buf[ii]);
        }

        t_last_packet_ = this->get_clock()->now();
    }

    // Done
    return;
}

bool transponder2ros::parseChar(unsigned char x)
{

    static uint  state        = 0;
    static uint  ii           = 0;
    static uint  expected_len = 0;
    static char  raw_buf[SIZEOF_MAX_UdpPacket];

    // State machine
    switch (state)
    {
    case (0): // Header 1
        ii = 0;
        if (x == xbee_headerA_)
        {
            state++;
        }
        break;

    case (1): // Header 2
        if (x == xbee_headerB_)
        {
            state++;
        }
        else
        {
            state = 0;
        }
        break;

    case (2): // Accumulate first 2 bytes: version + packet_type

        raw_buf[ii++] = x;

        if (ii == 2)
        {
            uint8_t pkt_type = static_cast<uint8_t>(raw_buf[1]);

            if (pkt_type == PACKET_TYPE_POSITION)
            {
                expected_len = SIZEOF_PositionUdpPacket;
                state++;
            }
            else if (pkt_type == PACKET_TYPE_COORDINATION)
            {
                expected_len = SIZEOF_CoordinationUdpPacket;
                state++;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown XBee packet type: 0x%02X", pkt_type);
                state = 0;
            }
        }

        break;

    case (3): // Accumulate remaining bytes

        raw_buf[ii++] = x;

        if (ii == expected_len)
        {
            state++;
        }

        break;

    case (4): // Check checksum
    {

        uint8_t crc8 = calc_crc8(raw_buf, expected_len);

        if (crc8 == x)
        {
            if (expected_len == SIZEOF_PositionUdpPacket)
            {
                PositionUdpPacket pos;
                std::memcpy(pos.raw, raw_buf, SIZEOF_PositionUdpPacket);
                publish_Transponder(pos);
            }
            else if (expected_len == SIZEOF_CoordinationUdpPacket)
            {
                CoordinationUdpPacket coord;
                std::memcpy(coord.raw, raw_buf, SIZEOF_CoordinationUdpPacket);
                publish_Coordination(coord);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                "Checksum failed.  Expected 0x%02X, got 0x%02X",
                x, crc8
            );
        }
        state = 0;
        break;
    }

    default:
        RCLCPP_WARN(this->get_logger(), "Invalid state reached, resetting");
        state = 0;
    }

    return (1);
}

uint8_t transponder2ros::calc_crc8(const char* data, size_t len)
{
    // https://www.analog.com/en/resources/technical-articles/
    // understanding-and-using-cyclic-redundancy-checks-with-
    // maxim-1wire-and-ibutton-products.html

    uint8_t crc = 0x00;

    for (size_t ii = 0; ii < len; ii++)
    {
        crc ^= data[ii];

        for (uint8_t jj = 0; jj < 8; jj++)
        {
            if (crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8C;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    // Return crc
    return crc;

}
