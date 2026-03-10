
// Transponder Structs
//  These should contain the same fields as transponder_msgs::Transponder.msg, however
//  may have different units / scales to optimise for size sent over the air.
//  Must be shared between the arduino and ROS2 nodes.

#include <stdint.h>

const uint8_t TRANSPONDER_UDP_STRUCT_VERISON = 0x05;  // 2025-06-28
                                                      // Check on the ROS side the versions of the
                                                      // structs you are getting are correct

// Packet type identifiers — byte 1 of every struct (after version)
const uint8_t PACKET_TYPE_POSITION     = 0x01;  // 10 Hz position data
const uint8_t PACKET_TYPE_COORDINATION = 0x02;  // 1 Hz pass coordination data

const char xbee_headerA_ = '$';
const char xbee_headerB_ = 'S';

// Vehicle state constants (state field in StructAVLTPosition)
const uint8_t STATE_UNKNOWN         = 0;
const uint8_t STATE_EMERGENCY_STOP  = 1;
const uint8_t STATE_CONTROLLED_STOP = 2;
const uint8_t STATE_NOMINAL         = 3;

// Pass state constants (pass_state field in StructAVLTCoordination)
const uint8_t PASS_STATE_IDLE         = 0;
const uint8_t PASS_STATE_REQUESTING   = 1;
const uint8_t PASS_STATE_ACKNOWLEDGED = 2;
const uint8_t PASS_STATE_PREPPING     = 3;
const uint8_t PASS_STATE_EXECUTING    = 4;
const uint8_t PASS_STATE_COMPLETED    = 5;
const uint8_t PASS_STATE_ABORTED      = 6;
const uint8_t PASS_STATE_SUSPENDED    = 7;

// 10 Hz: Vehicle position and state
struct __attribute__((packed)) StructAVLTPosition
{
  uint8_t  version;       // Struct version
  uint8_t  packet_type;   // = PACKET_TYPE_POSITION
  int32_t  sec;           // UTC time seconds [ s ]
  uint32_t nanosec;       // UTC time nanoseconds [ ns ]
  uint8_t  car_id;        // Car ID [ - ]
  uint8_t  heartbeat;     // Heartbeat counter [ - ], increments every frame, rolls over at 255
  int32_t  lat;           // Vehicle longitude [ dd.dd x 10^7 ]
  int32_t  lon;           // Vehicle latitude [ dd.dd x 10^7 ]
  int32_t  alt;           // Vehicle altitude [ mm ]
  uint16_t heading;       // Vehicle GPS heading [ cdeg ]
  uint16_t vel;           // Vehicle speed [ cm/s ]
  uint8_t  state;         // Vehicle state [ - ], see transponder_msgs::msg::Transponder
};

// 1 Hz: Pass coordination data
struct __attribute__((packed)) StructAVLTCoordination
{
  uint8_t  version;           // Struct version
  uint8_t  packet_type;       // = PACKET_TYPE_COORDINATION
  int32_t  sec;               // UTC time seconds [ s ]
  uint32_t nanosec;           // UTC time nanoseconds [ ns ]
  uint8_t  car_id;            // Car ID [ - ]
  uint8_t  pass_state;        // Pass FSM state [ - ], see PASS_STATE_* constants
  uint8_t  pass_sequence;     // Monotonic counter to correlate handshakes [ - ]
  uint8_t  target_car_id;     // Defender car ID being overtaken or followed [ - ]
  uint8_t  pass_zone_id;      // Identifier for the authorized straight where the pass occurs
  uint16_t yield_speed;       // Defender follow speed for yielding car [ cm/s ]
  uint16_t request_ttl_ms;    // Request time-to-live relative to header stamp [ ms ]
};

union PositionUdpPacket
{
  StructAVLTPosition data;
  char raw[sizeof(StructAVLTPosition)];
};

union CoordinationUdpPacket
{
  StructAVLTCoordination data;
  char raw[sizeof(StructAVLTCoordination)];
};

const uint SIZEOF_PositionUdpPacket     = sizeof(PositionUdpPacket);      // Position packet size
const uint SIZEOF_CoordinationUdpPacket = sizeof(CoordinationUdpPacket);  // Coordination packet size
const uint SIZEOF_MAX_UdpPacket         = sizeof(PositionUdpPacket);      // Largest packet type
