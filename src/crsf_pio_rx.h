#ifndef CRSF_PIO_RX_HEADER
#define CRSF_PIO_RX_HEADER

#include <stdbool.h>
#include <stdint.h>

#define CHANNEL_AMOUNT 16

typedef struct {
  bool crc_ok;
  uint16_t channels[CHANNEL_AMOUNT];
} crsf_packet;

void crsf_init(uint32_t pio_rx_pin, uint32_t baud);

void crsf_get_packet(crsf_packet *packet_buf);

#endif
