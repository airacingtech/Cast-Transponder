
// Transponder Structs
//  These should contain the same fields as transponder_msgs::Transponder.mgs, however
//  may have different units / scales to optimise for size sent over the air.
//  Must be shared between the arduino and ROS2 nodes.

#include <stdint.h>

const uint8_t TRANSPONDER_UDP_STRUCT_VERISON = 0x05;  // 2025-07-01
                                                      // Check on the ROS side the versions of the
                                                      // structs you are getting are correct

const char xbee_headerA_ = '$';
const char xbee_headerB_ = 'S';

struct __attribute__((packed)) StructIacTransponder
{
  uint8_t version;               // Struct version
  int32_t sec;                   // UTC time seconds [ s ]
  uint32_t nanosec;              // UTC time nanoseconds [ ns ]
  uint8_t car_id;                // Vehicle ID [ - ]
  uint8_t state;                 // Vehicle state [ - ]
  uint8_t heartbeat;             // Rolling heartbeat counter [ - ]
  int32_t lat_e7;                // Latitude * 1e7 [ signed deg * 1e7 ]
  int32_t lon_e7;                // Longitude * 1e7 [ signed deg * 1e7 ]
  int16_t alt_dm;                // Altitude in decimeters [ dm ]
  int16_t heading_cdeg;          // Heading centi-degrees [ deg * 100 ]
  int16_t vel_cms;               // Longitudinal speed [ cm/s ]
  uint8_t pass_state;            // Pass FSM state [ enum ]
  uint8_t pass_sequence;         // Pass handshake monotonic counter
  uint8_t target_car_id;         // Defender car ID being overtaken [ - ]
  uint8_t pass_zone_id;          // Authorized straight identifier
  uint16_t yield_speed_cms;      // Defender follow speed [ cm/s ]
  uint16_t request_ttl_ms;       // Request TTL relative to header.stamp [ ms ]
};

union TransponderUdpPacket
{
  StructIacTransponder data;
  char raw[sizeof(StructIacTransponder)];
};

const uint SIZEOF_TransponderUdpPacket = sizeof(TransponderUdpPacket);
