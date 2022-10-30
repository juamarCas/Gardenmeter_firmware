#ifndef PAYLOAD_H
#define PAYLOAD
#include <cstdint>

typedef struct Payload_str{
  std::uint16_t moist;
  float env_temp;
  float env_hum;
  std::uint16_t soil_temp;
  std::uint16_t light;
}Payload;

#endif