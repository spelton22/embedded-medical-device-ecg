#include "calc_cycles.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(main, LOG_LEVEL_DBG);

int calc_cycles(int16_t *buffer, int buffer_size){

  int threshold = 300;
  int cycles = 0;

  for (int i = 0; i < buffer_size; i++) {
    if(buffer[i] > threshold && buffer[i-1] < threshold) {
      cycles++;
    }
  }

  return cycles;
}
