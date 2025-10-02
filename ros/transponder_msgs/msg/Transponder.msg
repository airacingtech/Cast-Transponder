std_msgs/Header header        # Standard ROS header (stamp drives relative timing)

uint8   car_id                # Vehicle ID [ - ]
uint8   state                 # Vehicle state enum below
uint8   heartbeat             # Rolling heartbeat counter; drop rate exposes link quality [ - ]

int32   lat_e7                # Latitude * 1e7, rear-axle centre [ signed deg * 1e7 ]
int32   lon_e7                # Longitude * 1e7, rear-axle centre [ signed deg * 1e7 ]
int16   alt_dm                # Altitude (ellipsoid) in decimetres [ dm ]
int16   heading_cdeg          # Heading in centi-degrees, North = 0 [ deg * 100 ]
int16   vel_cms               # Longitudinal speed in centimetres per second [ cm/s ]

uint8   pass_state            # Engagement finite-state machine value [ enum below ]
uint8   pass_sequence         # Monotonic counter to correlate handshakes
uint8   target_car_id         # Defender car ID being overtaken or followed [ - ]
uint8   pass_zone_id          # Identifier for the authorized straight where the pass occurs
uint16  yield_speed_cms       # Defender follow speed for yielding car [ cm/s ]
uint16  request_ttl_ms        # Request time-to-live relative to header.stamp [ ms ]

# Vehicle state constants
uint8 STATE_UNKNOWN = 0
uint8 STATE_EMERGENCY_STOP = 1
uint8 STATE_CONTROLLED_STOP = 2
uint8 STATE_NOMINAL = 3

# Pass state constants
uint8 PASS_STATE_IDLE = 0
uint8 PASS_STATE_REQUESTING = 1
uint8 PASS_STATE_ACKNOWLEDGED = 2
uint8 PASS_STATE_PREPPING = 3
uint8 PASS_STATE_EXECUTING = 4
uint8 PASS_STATE_COMPLETED = 5
uint8 PASS_STATE_ABORTED = 6
uint8 PASS_STATE_SUSPENDED = 7
