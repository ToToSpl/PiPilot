#include "crsf_pio_rx.h"
#include "hardware/pio.h"
#include "uart_rx.pio.h"

typedef struct {
  PIO pio;
  uint sm;
  int8_t pio_irq;
  uint offset;
} crsf_pio_rx_private_t;

static crsf_pio_rx_private_t state;

// IRQ called when the pio fifo is not empty, i.e. there are some characters on the uart
// This needs to run as quickly as possible or else you will lose characters (in particular don't
// printf!)
static void pio_irq_func(void) {
  while (!pio_sm_is_rx_fifo_empty(state.pio, state.sm)) {
    char c = uart_rx_program_getc(state.pio, state.sm);
    // do stuff with c
    printf("read: %X\n", c);
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

void crsf_init() {
  // Set up the state machine we're going to use to receive them.
  // In real code you need to find a free pio and state machine in case pio resources are used
  // elsewhere
  if (!init_pio(&uart_rx_program, &state.pio, &state.sm, &state.offset)) {
    panic("failed to setup pio");
  }
  uart_rx_program_init(state.pio, state.sm, state.offset, PIO_RX_PIN, SERIAL_BAUD);

  // Find a free irq
  static_assert(PIO0_IRQ_1 == PIO0_IRQ_0 + 1 && PIO1_IRQ_1 == PIO1_IRQ_0 + 1, "");
  state.pio_irq = (state.pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
  if (irq_get_exclusive_handler(state.pio_irq)) {
    state.pio_irq++;
    if (irq_get_exclusive_handler(state.pio_irq)) {
      panic("All IRQs are in use");
    }
  }

  // Enable interrupt
  irq_add_shared_handler(
      state.pio_irq, pio_irq_func,
      PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY); // Add a shared IRQ handler
  irq_set_enabled(state.pio_irq, true);                // Enable the IRQ
  const uint irq_index =
      state.pio_irq - ((state.pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0); // Get index of the IRQ
  pio_set_irqn_source_enabled(state.pio, irq_index, pis_sm0_rx_fifo_not_empty + state.sm,
                              true); // Set pio to tell us when the FIFO is NOT empty
}
