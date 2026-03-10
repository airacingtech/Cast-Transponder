
#include <ETH.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "iac_udp_struct.h"
#include "xbee_struct.h"

// Team Defines (comment / uncomment as required)

#define TRANSPONDER_IP "10.42.17.210"  // #7 (Berkeley)
#define COMPUTER_IP "10.42.17.200"

// #define TRANSPONDER_IP "10.42.8.60"  // #8 (Caltech Racer)
// #define COMPUTER_IP "10.42.8.4"

// IP settings
IPAddress local_IP(TRANSPONDER_IP);        // IP Address of the Transponder
IPAddress ip_send_(COMPUTER_IP);           // Destination IP for UDP messages (ROS2 computer)
const unsigned int port_ = 15783;          // UDP port  15783

// Globals
const bool print_debug_ = 0;               // Disable debug prints unless actively using
WiFiUDP udp;

IPAddress gateway(COMPUTER_IP);
IPAddress subnet("255.255.255.0");
IPAddress dns("8.8.8.8");

static bool eth_connected = false;
int32_t last_udp_sec_ = 0;
uint32_t last_udp_nanosec_ = 0;

char buf_[100];   // Buffer for printing strings

// React to Ethernet events:
void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {

    case ARDUINO_EVENT_ETH_START:
      // This will happen during setup, when the Ethernet service starts
      Serial.println("ETH Started");
      ETH.config(local_IP, gateway, subnet, dns);
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      // This will happen when the Ethernet cable is plugged
      Serial.println("ETH Connected");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
    // This will happen when we obtain an IP address through DHCP:
      Serial.print("Got an IP Address for ETH MAC: ");
      Serial.println(ETH.macAddress());
      Serial.print("  IPv4: ");
      Serial.println(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(" FULL_DUPLEX");
      }
      Serial.print("  ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      Serial.print("Sending data to ");
      Serial.println(ip_send_);
      eth_connected = true;
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      // This will happen when the Ethernet cable is unplugged
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;

    case ARDUINO_EVENT_ETH_STOP:
      // This will happen when the ETH interface is stopped but this never happens
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;

    default:
      break;
  }
}

// Initializing everything at start up / after reset:
void setup()
{
  // Wait for the hardware to initialize:
  delay(500);

  // Start debugging serial port
  Serial.begin(115200);

  Serial.println("=====================");
  Serial.println("IAC Transponder System");
  Serial.println("TRANSPONDER_IP: "+String(TRANSPONDER_IP)+", COMPUTER_IP: "+String(COMPUTER_IP));
  Serial.println("  Transponder Struct Version: "+String(TRANSPONDER_UDP_STRUCT_VERISON)
    +", len(Position): "+String(SIZEOF_PositionUdpPacket)+" bytes"
    +", len(Coordination): "+String(SIZEOF_CoordinationUdpPacket)+" bytes");
  Serial.println("=====================");
  Serial.print("Setup...");

  // Start Xbee serial port
  Serial1.begin(115200);

  // Add a handler for network events. This is misnamed "WiFi" because the ESP32 is historically WiFi only,
  // but in our case, this will react to Ethernet events.
  Serial.print("Registering event handler for ETH events...");
  WiFi.onEvent(WiFiEvent);

  // Starth Ethernet (this does NOT start WiFi at the same time)
  Serial.print("Starting ETH interface...");
  ETH.begin();

  udp.begin(port_);

}

void loop()
{

  process_udp();
  process_xbee();
  // process_debug();  // Code for sending debug packets

}

void process_udp()
{
  // Receive UDP and send to Serial1
  int packetSize = udp.parsePacket();

  if (packetSize == SIZEOF_PositionUdpPacket)
  {
    PositionUdpPacket pos_pkt;
    int len = udp.read(pos_pkt.raw, SIZEOF_PositionUdpPacket);

    // Debugging
    if (print_debug_)
    {
      Serial.print("\nReceived position from UDP / send XBee\n");
      sprintf(buf_, "    Version: %d\n"    , pos_pkt.data.version);    Serial.print(buf_);
      sprintf(buf_, "       Type: 0x%02X\n", pos_pkt.data.packet_type); Serial.print(buf_);
      sprintf(buf_, "        sec: %d\n"    , pos_pkt.data.sec);         Serial.print(buf_);
      sprintf(buf_, "    nanosec: %d\n"    , pos_pkt.data.nanosec);     Serial.print(buf_);
      sprintf(buf_, "        Car: %d\n"    , pos_pkt.data.car_id);      Serial.print(buf_);
      sprintf(buf_, "      HBeat: %d\n"    , pos_pkt.data.heartbeat);   Serial.print(buf_);
      sprintf(buf_, "        Lat: %11.5f\n", pos_pkt.data.lat/1e7);     Serial.print(buf_);
      sprintf(buf_, "        Lon: %11.5f\n", pos_pkt.data.lon/1e7);     Serial.print(buf_);
      sprintf(buf_, "        Alt: %11.5f\n", pos_pkt.data.alt/1e3);     Serial.print(buf_);
      sprintf(buf_, "    Heading: %5.2f\n" , pos_pkt.data.heading/1e2); Serial.print(buf_);
      sprintf(buf_, "        Vel: %5.2f\n" , pos_pkt.data.vel/1e2);     Serial.print(buf_);
      sprintf(buf_, "      State: %d\n"    , pos_pkt.data.state);       Serial.print(buf_);
    }

    if (len == SIZEOF_PositionUdpPacket)
    {
      forward_to_xbee(pos_pkt.raw, SIZEOF_PositionUdpPacket);

      // Store the last UTC time a position packet was received
      last_udp_sec_ = pos_pkt.data.sec;
      last_udp_nanosec_ = pos_pkt.data.nanosec;
    }
    else
    {
      sprintf(buf_, "Read incorrect number of bytes. Got %d, expected %d\n", len, SIZEOF_PositionUdpPacket);
      Serial.print(buf_);
    }
  }
  else if (packetSize == SIZEOF_CoordinationUdpPacket)
  {
    CoordinationUdpPacket coord_pkt;
    int len = udp.read(coord_pkt.raw, SIZEOF_CoordinationUdpPacket);

    // Debugging
    if (print_debug_)
    {
      Serial.print("\nReceived coordination from UDP / send XBee\n");
      sprintf(buf_, "    Car: %d, PassState: %d, Seq: %d, Target: %d\n",
        coord_pkt.data.car_id, coord_pkt.data.pass_state,
        coord_pkt.data.pass_sequence, coord_pkt.data.target_car_id);
      Serial.print(buf_);
    }

    if (len == SIZEOF_CoordinationUdpPacket)
    {
      forward_to_xbee(coord_pkt.raw, SIZEOF_CoordinationUdpPacket);
    }
    else
    {
      sprintf(buf_, "Read incorrect number of bytes. Got %d, expected %d\n", len, SIZEOF_CoordinationUdpPacket);
      Serial.print(buf_);
    }
  }
  else if (packetSize)
  {
    sprintf(buf_, "Packet size received incorrect. Got %d, expected %d or %d\n",
      packetSize, SIZEOF_PositionUdpPacket, SIZEOF_CoordinationUdpPacket);
    Serial.print(buf_);
  }

  // Done
  return;

}

void process_xbee()
{

  while (Serial1.available())
  {

    // Send data through state machine
    xbee_state_machine(Serial1.read());

  }

}

void process_debug()
{
  // Send debug data
  static unsigned long t_last = millis();

  if (millis() > t_last + 100)
  {
    send_test_udp();
    t_last = millis();
  }

  if (0)
  {
    udp.beginPacket(ip_send_, port_);
    udp.print("hello");
    udp.endPacket();
    delay(50);
  }
}

void forward_to_xbee(const char* data, size_t len)
{
  // Wrap payload with XBee framing: header ($S) + data + CRC8
  uint8_t crc8 = calc_crc8(data, len);
  Serial1.write((uint8_t)xbee_headerA_);
  Serial1.write((uint8_t)xbee_headerB_);
  Serial1.write((const uint8_t*)data, len);
  Serial1.write(crc8);
}

void xbee_state_machine(char x)
{
  static uint  state        = 0;
  static uint  ii           = 0;
  static uint  expected_len = 0;
  static char  raw_buf[SIZEOF_MAX_UdpPacket];

  switch (state)
  {
    case (0) :
      {
        // Reset variables and wait for first header byte
        ii = 0;
        if (x == xbee_headerA_)
          state++;
      }
      break;

    case (1) :
      {
        if (x == xbee_headerB_)
        {
          state++;
        }
        else
        {
          state = 0;
        }
      }
      break;

    case (2) :
      {
        // Accumulate first 2 bytes: version + packet_type
        raw_buf[ii++] = x;

        if (ii == 2)
        {
          uint8_t pkt_type = (uint8_t)raw_buf[1];

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
            sprintf(buf_, "Unknown XBee packet type: 0x%02X\n", pkt_type);
            Serial.print(buf_);
            state = 0;
          }
        }
      }
      break;

    case (3) :
      {
        // Accumulate remaining bytes until we reach expected_len
        raw_buf[ii++] = x;

        if (ii == expected_len)
          state++;
      }
      break;

    case (4) :
      {
        // Check CRC, then forward via UDP
        uint8_t crc8 = calc_crc8(raw_buf, expected_len);

        if (x == crc8)
        {
          udp.beginPacket(ip_send_, port_);
          udp.write((uint8_t*)raw_buf, expected_len);
          udp.endPacket();

          // Debugging: print position fields if enabled
          if (print_debug_ && expected_len == SIZEOF_PositionUdpPacket)
          {
            StructAVLTPosition* pos = (StructAVLTPosition*)raw_buf;
            Serial.print("\nReceived position from XBee / Send UDP\n");
            sprintf(buf_, "    Version: %d\n"    , pos->version);        Serial.print(buf_);
            sprintf(buf_, "        sec: %d\n"    , pos->sec);             Serial.print(buf_);
            sprintf(buf_, "    nanosec: %d\n"    , pos->nanosec);         Serial.print(buf_);
            sprintf(buf_, "        Car: %d\n"    , pos->car_id);          Serial.print(buf_);
            sprintf(buf_, "      HBeat: %d\n"    , pos->heartbeat);       Serial.print(buf_);
            sprintf(buf_, "        Lat: %13.8f\n", pos->lat/1e7);         Serial.print(buf_);
            sprintf(buf_, "        Lon: %13.8f\n", pos->lon/1e7);         Serial.print(buf_);
            sprintf(buf_, "        Alt: %8.3f\n" , pos->alt/1e3);         Serial.print(buf_);
            sprintf(buf_, "    Heading: %5.2f\n" , pos->heading/1e2);     Serial.print(buf_);
            sprintf(buf_, "        Vel: %5.2f\n" , pos->vel/1e2);         Serial.print(buf_);
            sprintf(buf_, "      State: %d\n"    , pos->state);           Serial.print(buf_);
            Serial.print("\n");
          }
        }
        else
        {
          sprintf(buf_, "Checksum error.  Expected 0x%02X, got 0x%02X\n", crc8, x);
          Serial.print(buf_);
          Serial.println("  If this happens every frame, check the packet versions match");
        }

        // Reset state machine
        state = 0;
      }
      break;
  }
}

void send_test_udp()
{

  static PositionUdpPacket test_data;

  // Fill the packet with some random position data
  test_data.data.version    = TRANSPONDER_UDP_STRUCT_VERISON;
  test_data.data.packet_type = PACKET_TYPE_POSITION;
  test_data.data.sec        = last_udp_sec_;
  test_data.data.nanosec    = last_udp_nanosec_;
  test_data.data.car_id     = 1;
  test_data.data.heartbeat  = 0;

  test_data.data.lat     =   362680800 + random(-10, 10);
  test_data.data.lon     = -1150181860 + random(-10, 10);
  test_data.data.alt     =  142000*1e3 + random(-100, 100);
  test_data.data.heading = random(0, 36000);
  test_data.data.vel     = random(0, 10000);
  test_data.data.state   = 3;

  // Send the packet
  udp.beginPacket(ip_send_, port_);
  udp.write((uint8_t*)test_data.raw, SIZEOF_PositionUdpPacket);
  udp.endPacket();

}

uint8_t calc_crc8(const char* data, size_t len)
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
