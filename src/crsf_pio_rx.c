#include "crsf_pio_rx.h"
#include "hardware/pio.h"
#include "uart_rx.pio.h"
#include <stdint.h>

#define PACKET_START 0xC8
#define PACKET_RC_LEN 0x18
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define PACKET_SIZE                                                                                \
  PACKET_RC_LEN + 2 // START, LEN, TYPE, PAYLOAD, CRC8, type and crc8 included in LEN

PIO pio;
uint sm;
int8_t pio_irq;
uint offset;

uint8_t *data_buf_read[PACKET_SIZE];
uint8_t *data_buf_write[PACKET_SIZE];
uint8_t buf_index = 0;
bool read_started = false;

// IRQ called when the pio fifo is not empty, i.e. there are some characters on the uart
// This needs to run as quickly as possible or else you will lose characters (in particular don't
// printf!)
static void pio_irq_func(void) {
  while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
    char c = uart_rx_program_getc(pio, sm);
    if (c == PACKET_START) {
      buf_index = 0;
      read_started = true;
    }
    if (!read_started)
      continue;

    (*data_buf_write)[buf_index++] = c;

    if (buf_index == PACKET_SIZE) {
      uint8_t *tmp = *data_buf_read;
      *data_buf_read = *data_buf_write;
      *data_buf_write = tmp;

      buf_index = 0;
      read_started = false;
    }
  }
}

// Find a free pio and state machine and load the program into it.
// Returns false if this fails
static bool init_pio(const pio_program_t *program, PIO *pio_hw, uint *sm, uint *offset) {
  // Find a free pio
  *pio_hw = pio1;
  if (!pio_can_add_program(*pio_hw, program)) {
    *pio_hw = pio0;
    if (!pio_can_add_program(*pio_hw, program)) {
      *offset = -1;
      return false;
    }
  }
  *offset = pio_add_program(*pio_hw, program);
  // Find a state machine
  *sm = (int8_t)pio_claim_unused_sm(*pio_hw, false);
  if (*sm < 0) {
    return false;
  }
  return true;
}

void crsf_init(uint32_t pio_rx_pin, uint32_t baud) {
  // Set up the state machine we're going to use to receive them.
  // In real code you need to find a free pio and state machine in case pio resources are used
  // elsewhere
  if (!init_pio(&uart_rx_program, &pio, &sm, &offset)) {
    panic("failed to setup pio");
  }
  uart_rx_program_init(pio, sm, offset, pio_rx_pin, baud);

  // Find a free irq
  static_assert(PIO0_IRQ_1 == PIO0_IRQ_0 + 1 && PIO1_IRQ_1 == PIO1_IRQ_0 + 1, "");
  pio_irq = (pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
  if (irq_get_exclusive_handler(pio_irq)) {
    pio_irq++;
    if (irq_get_exclusive_handler(pio_irq)) {
      panic("All IRQs are in use");
    }
  }

  // Enable interrupt
  irq_add_shared_handler(
      pio_irq, pio_irq_func,
      PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY); // Add a shared IRQ handler
  irq_set_enabled(pio_irq, true);                      // Enable the IRQ
  const uint irq_index =
      pio_irq - ((pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0); // Get index of the IRQ
  pio_set_irqn_source_enabled(pio, irq_index, pis_sm0_rx_fifo_not_empty + sm,
                              true); // Set pio to tell us when the FIFO is NOT empty
}

static void unpackChannels(uint8_t const *const payload, uint16_t *const dest) {
  const unsigned numOfChannels = 16;
  const unsigned srcBits = 11;
  const unsigned dstBits = 32;
  const unsigned inputChannelMask = (1 << srcBits) - 1;

  // code from BetaFlight rx/crsf.cpp / bitpacker_unpack
  uint8_t bitsMerged = 0;
  uint32_t readValue = 0;
  unsigned readByteIndex = 0;
  for (uint8_t n = 0; n < numOfChannels; n++) {
    while (bitsMerged < srcBits) {
      uint8_t readByte = payload[readByteIndex++];
      readValue |= ((uint32_t)readByte) << bitsMerged;
      bitsMerged += 8;
    }
    // printf("rv=%x(%x) bm=%u\n", readValue, (readValue & inputChannelMask), bitsMerged);
    dest[n] = (uint16_t)(readValue & inputChannelMask);
    readValue >>= srcBits;
    bitsMerged -= srcBits;
  }
}

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
  static const uint8_t crsf_crc8tab[256] = {
      0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8,
      0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50,
      0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73,
      0xA6, 0x0C, 0xD9, 0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75,
      0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB,
      0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33,
      0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2, 0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10,
      0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
      0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E,
      0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92, 0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96,
      0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5,
      0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3,
      0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64, 0x72, 0xA7, 0x0D,
      0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5,
      0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6,
      0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
      0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C,
      0xF9};

  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = crsf_crc8tab[crc ^ *ptr++];
  }
  return crc;
}

void crsf_get_packet(crsf_packet *packet_buf) {
  // first check if packet is of correct type
  {
    bool check = (*data_buf_read)[0] == PACKET_START && (*data_buf_read)[1] == PACKET_RC_LEN &&
                 (*data_buf_read)[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    if (!check) {
      packet_buf->crc_ok = false;
      return;
    }
  }

  unpackChannels(&(*data_buf_read)[3], packet_buf->channels);
  uint8_t crc = crsf_crc8(&(*data_buf_read)[2], PACKET_RC_LEN + 1);
  packet_buf->crc_ok = crc == (*data_buf_read)[PACKET_SIZE - 1];
}
