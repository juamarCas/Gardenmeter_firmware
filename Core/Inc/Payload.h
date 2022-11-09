#ifndef PAYLOAD_H
#define PAYLOAD
#include <cstdint>

#pragma pack(1)
typedef struct Payload_str{
  std::uint16_t id;
  std::uint8_t moist;
  float env_temp;
  float env_hum;
  std::uint16_t soil_temp;
  std::uint8_t light;
} Payload;


#endif

