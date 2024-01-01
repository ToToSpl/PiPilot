#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

void main_core1() {
  sleep_ms(500);

  while (1) {
    printf("Hello World2\n");

    sleep_ms(1000);
  }
}

int main() {

  stdio_init_all();
  setup_default_uart();

  multicore_launch_core1(main_core1);

  while (1) {
    printf("Hello World1!\n");

    sleep_ms(1000);
  }

  return 0;
}
