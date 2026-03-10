# Pass coordination message for multi-car engagement (1 Hz)
# See iac_udp_struct.h for PASS_STATE_* and PACKET_TYPE_* constants.

std_msgs/Header header        # Standard ROS header (stamp drives relative timing)

uint8   car_id                # Vehicle ID [ - ]
uint8   pass_state            # Engagement finite-state machine value (PASS_STATE_* below) [ - ]
uint8   pass_sequence         # Monotonic counter to correlate handshakes [ - ]
uint8   target_car_id         # Defender car ID being overtaken or followed [ - ]
uint8   pass_zone_id          # Identifier for the authorized straight where the pass occurs [ - ]
float32 yield_speed           # Defender follow speed for yielding car [ m/s ]
uint16  request_ttl_ms        # Request time-to-live relative to header.stamp [ ms ]

# Pass state constants
uint8 PASS_STATE_IDLE         = 0
uint8 PASS_STATE_REQUESTING   = 1
uint8 PASS_STATE_ACKNOWLEDGED = 2
uint8 PASS_STATE_PREPPING     = 3
uint8 PASS_STATE_EXECUTING    = 4
uint8 PASS_STATE_COMPLETED    = 5
uint8 PASS_STATE_ABORTED      = 6
uint8 PASS_STATE_SUSPENDED    = 7
