
#include <ETH.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "iac_udp_struct.h"
#include "xbee_struct.h"

// Team Defines (comment / uncomment as required)

#define TRANSPONDER_IP "10.42.7.210"  // #7 (Berkeley)
#define COMPUTER_IP "10.42.7.200"

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
  Serial.println("  Transponder Struct Version: "+String(TRANSPONDER_UDP_STRUCT_VERISON)+", len(XBee Packet): "+String(SIZEOF_XbeePacket)+" bytes");
  Serial.println("=====================");
  Serial.print("Setup...");

  // Start Xbee serial port
  Serial1.begin(57600);
  
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

  if (packetSize == SIZEOF_TransponderUdpPacket)
  {
    // sprintf(buf_,"%8d | UDP packet received (%d bytes)\n",millis(),packetSize); 
    // Serial.print(buf_);

    XbeePacket xbee_packet;

    // Copy UDP packet into Xbee struct
    int len = udp.read(xbee_packet.data.data.raw, SIZEOF_TransponderUdpPacket);

    // Debugging
    if (print_debug_)
    {
      Serial.print("\nReceived from UDP / send XBee\n");
      sprintf(buf_, "    Version: %d\n"    ,xbee_packet.data.data.data.version); Serial.print(buf_);
      sprintf(buf_, "        sec: %d\n"    ,xbee_packet.data.data.data.sec); Serial.print(buf_);
      sprintf(buf_, "    nanosec: %d\n"    ,xbee_packet.data.data.data.nanosec); Serial.print(buf_);
      sprintf(buf_, "        Car: %d\n"    ,xbee_packet.data.data.data.car_id); Serial.print(buf_);
      sprintf(buf_, "        Lat: %11.5f\n",xbee_packet.data.data.data.lat/1e7); Serial.print(buf_);
      sprintf(buf_, "        Lon: %11.5f\n",xbee_packet.data.data.data.lon/1e7); Serial.print(buf_);
      sprintf(buf_, "        Alt: %11.5f\n",xbee_packet.data.data.data.alt/1e3); Serial.print(buf_);
      sprintf(buf_, "    Heading: %5.2f\n" ,xbee_packet.data.data.data.heading/1e2); Serial.print(buf_);
      sprintf(buf_, "        Vel: %5.2f\n" ,xbee_packet.data.data.data.vel/1e2); Serial.print(buf_);
      sprintf(buf_, "      State: %d\n"    ,xbee_packet.data.data.data.state); Serial.print(buf_);
    }

    if (len == SIZEOF_TransponderUdpPacket)
    {
        // Headers
        xbee_packet.data.headerA = xbee_headerA_;
        xbee_packet.data.headerB = xbee_headerB_;

        // Calculate checksum for Xbee struct.  Only use the actual struct data as I loose the headers in the decode
        xbee_packet.data.crc8 = calc_crc8(xbee_packet.data.data.raw, SIZEOF_TransponderUdpPacket);

        // Forward packet out via Serial1
        Serial1.write((uint8_t*)xbee_packet.raw, SIZEOF_XbeePacket);

        // Store the last UTC time a packet was received
        last_udp_sec_ = xbee_packet.data.data.data.sec;
        last_udp_nanosec_ = xbee_packet.data.data.data.nanosec;
    }
    else
    {
      sprintf(buf_,"Read incorrect number of bytes. Got %d, expected %d\n",len,SIZEOF_TransponderUdpPacket); 
      Serial.print(buf_);
    }
  }
  else if (packetSize)
  {

    sprintf(buf_,"Packet size received incorrect. Got %d, expected %d\n",packetSize,SIZEOF_TransponderUdpPacket); 
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

void xbee_state_machine(char x)
{
  if (0)
  {
    sprintf(buf_,"0x%02X ",x);
    Serial.print(buf_);
  }

  static uint state = 0;
  static uint ii = 0;
  static TransponderUdpPacket data;

  switch (state)
  {
    case (0) :
      {
        // Reset variables
        ii = 0;
        // sprintf(buf_,"Looking for headerA : 0x%02X\n",headerA);
        // Serial.print(buf_);

        // Check for header
        if (x == xbee_headerA_)
          state++;
      } 
      break;
    case (1) :
      {
        // sprintf(buf_,"Looking for headerB : 0x%02X\n",headerB);
        // Serial.print(buf_);

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

        data.raw[ii] = x;
        ii++;

        if (ii == SIZEOF_TransponderUdpPacket)
          state++;

      }
      break;
    case (3) :
      {
        // Serial.print("Checking checksum\n");
        uint8_t crc8 = calc_crc8(data.raw, SIZEOF_TransponderUdpPacket);  // Don't include header in checksum
        
        if (x == crc8)
        {
          // Data good, send via UDP
          // Serial.print("Sending UDP packet\n");

          udp.beginPacket(ip_send_, port_);
          udp.write((uint8_t*)data.raw, SIZEOF_TransponderUdpPacket);
          udp.endPacket();

          // Debugging
          if (print_debug_)
          {
            Serial.print("\nReceived from XBee / Send UDP\n");
            sprintf(buf_, "    Version: %d\n"    ,data.data.version); Serial.print(buf_);
            sprintf(buf_, "        sec: %d\n"    ,data.data.sec); Serial.print(buf_);
            sprintf(buf_, "    nanosec: %d\n"    ,data.data.nanosec); Serial.print(buf_);
            sprintf(buf_, "        Car: %d\n"    ,data.data.car_id); Serial.print(buf_);
            sprintf(buf_, "        Lat: %13.8f\n",data.data.lat/1e7); Serial.print(buf_);
            sprintf(buf_, "        Lon: %13.8f\n",data.data.lon/1e7); Serial.print(buf_);
            sprintf(buf_, "        Alt: %8.3f\n" ,data.data.alt/1e3); Serial.print(buf_);
            sprintf(buf_, "    Heading: %5.2f\n" ,data.data.heading/1e2); Serial.print(buf_);
            sprintf(buf_, "        Vel: %5.2f\n" ,data.data.vel/1e2); Serial.print(buf_);
            sprintf(buf_, "      State: %d\n"    ,data.data.state); Serial.print(buf_);
            Serial.print("\n");
          }
        } 
        else
        {
          sprintf(buf_, "Checksum error.  Expected 0x%02X, got 0x%02X\n",crc8,x);
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

  static TransponderUdpPacket test_data;

  // Fill the packet with some random stuff
  test_data.data.version = TRANSPONDER_UDP_STRUCT_VERISON;
  test_data.data.sec = last_udp_sec_;
  test_data.data.nanosec = last_udp_nanosec_;
  test_data.data.car_id = 1;

  test_data.data.lat =   362680800 + random(-10, 10);
  test_data.data.lon = -1150181860 + random(-10, 10);
  test_data.data.alt =  142000*1e3 + random(-100, 100);
  test_data.data.heading = random(0,36000);
  test_data.data.vel = random(0,10000);
  test_data.data.state = 3;

  // Send the packet
  udp.beginPacket(ip_send_, port_);
  udp.write((uint8_t*)test_data.raw, SIZEOF_TransponderUdpPacket);
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
