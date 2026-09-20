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
    // RCLCPP_INFO(this->get_logger(), "[DEVICE_PATH]: %s", device_path.c_str());
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
    // struct termios tty_old;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(p_serialPort, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
        rclcpp::shutdown();
        return false;
    }

    /* Save old tty parameters */
    // tty_old = tty;

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
        
        // Frames from here are tagged kSerialRadioTap, so they publish car data but never move
        // the link clock. This dongle is wired straight to the host and bypasses the transponder
        // box, so it proves another car is transmitting, not that our own unit is reachable --
        // which is the whole claim the published link status makes.
        for (int ii = 0; ii < n_read; ii++)
        {
            parseChar(buf[ii]);
        }
    }

    // Done
    return;
}

bool transponder2ros::parseChar(unsigned char x)
{

    static uint state = 0;
    static uint ii = 0;
    static TransponderUdpPacket data;

    // State machine
    switch (state)
    {
    case (0): // Header 1
        if (x == xbee_headerA_)
        {
            state++;
        }
        break;
    
    case (1): // Header 2
        if (x == xbee_headerB_)
        {
            state++;
            ii = 0;
        }
        else
        {
            state = 0;
        }
        break;

    case (2): // Store message

        data.raw[ii] = x;
        ii++;

        if (ii == SIZEOF_TransponderUdpPacket)
        {
            // RCLCPP_INFO(this->get_logger(), "Data packet received");
            state++;
        }

        break;

    case (3): // Check checksum
    {

        // for (size_t i = 0; i < SIZEOF_TransponderUdpPacket; ++i) {
        //     printf("%02X ", (uint8_t)data.raw[i]);
        // }
        // printf("\n");

        uint8_t crc8 = calc_crc8(data.raw, SIZEOF_TransponderUdpPacket);  // Don't include header in checksum

        if (crc8 == x)
        {
            // Checksum passed, assemble and publish message
            // RCLCPP_INFO(this->get_logger(), "Checksum passed");
            publish_Transponder(data, PacketSource::kSerialRadioTap);
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
