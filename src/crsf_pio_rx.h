#ifndef CRSF_PIO_RX_HEADER
#define CRSF_PIO_RX_HEADER

#include <stdbool.h>
#include <stdint.h>

#define PIO_RX_PIN 22
#define SERIAL_BAUD 9600 / 8
#define CHANNEL_AMOUNT 8

typedef struct {
  uint16_t channels[CHANNEL_AMOUNT];
} crsf_packet;

void crsf_init();

void crsf_get_packet(crsf_packet *packet_buf);

#endif
