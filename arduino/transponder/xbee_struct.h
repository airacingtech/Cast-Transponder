
#include <stdint.h>

struct __attribute__((packed)) StructXbeePosition
{
  char headerA;               // Header '$'
  char headerB;               // Header 'S'
  PositionUdpPacket data;     // Position payload
  uint8_t crc8;               // CRC8 checksum of data only (not headers)
};

struct __attribute__((packed)) StructXbeeCoordination
{
  char headerA;               // Header '$'
  char headerB;               // Header 'S'
  CoordinationUdpPacket data; // Coordination payload
  uint8_t crc8;               // CRC8 checksum of data only (not headers)
};

union XbeePositionPacket
{
  StructXbeePosition data;
  char raw[sizeof(StructXbeePosition)];
};

union XbeeCoordinationPacket
{
  StructXbeeCoordination data;
  char raw[sizeof(StructXbeeCoordination)];
};

const uint SIZEOF_XbeePositionPacket     = sizeof(XbeePositionPacket);
const uint SIZEOF_XbeeCoordinationPacket = sizeof(XbeeCoordinationPacket);
