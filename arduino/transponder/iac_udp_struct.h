
// Transponder Structs
//  These should contain the same fields as transponder_msgs::Transponder.mgs, however
//  may have different units / scales to optimise for size sent over the air.
//  Must be shared between the arduino and ROS2 nodes.

#include <stdint.h>

const uint8_t TRANSPONDER_UDP_STRUCT_VERISON = 0x04;  // 2025-06-28
                                                      // Check on the ROS side the versions of the
                                                      // structs you are getting are correct

const char xbee_headerA_ = '$';
const char xbee_headerB_ = 'S';

// car_id sentinel for a local liveness heartbeat -- proves the transponder's Ethernet link and
// firmware loop are alive even when nothing has been heard over the XBee radio. Matches
// race_msgs::msg::VehicleId::UNKNOWN (0), which is already reserved as "no car" -- no real
// competitor is ever numbered 0, so this is not a new allocation, just reusing the existing one.
const uint8_t HEARTBEAT_CAR_ID = 0;

struct __attribute__((packed)) StructIacTransponder
{
  uint8_t version;               // Struct version
  int32_t sec;                   // UTC time seconds [ s ]
  uint32_t nanosec;              // UTC time nanoseconds [ ns ]
  uint8_t car_id;                // Car ID [ - ]
  int32_t lat;                   // Vehicle longitude [ dd.dd x 10^7 ]
  int32_t lon;                   // Vehicle latitude [ dd.dd x 10^7 ]
  int32_t alt;                   // Vehicle altitude [ mm ]
  uint16_t heading;              // Vehicle GPS heading [ cdeg ]
  uint16_t vel;                  // Vehicle speed [ cm/s ]
  uint8_t state;                 // Vehicle state [ - ], see transponder_msgs::msg::Transponder
};

union TransponderUdpPacket
{
  StructIacTransponder data;
  char raw[sizeof(StructIacTransponder)];
};

const uint SIZEOF_TransponderUdpPacket = sizeof(TransponderUdpPacket);
