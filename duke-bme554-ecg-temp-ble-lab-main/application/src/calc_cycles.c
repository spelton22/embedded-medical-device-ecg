#include "calc_cycles.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(main, LOG_LEVEL_DBG);

int calc_cycles(int16_t *buffer, int buffer_size) {
  // Find min and max of the buffer
  int16_t max_val = buffer[0];
  int16_t min_val = buffer[0];
  
  for (int i = 1; i < buffer_size; i++) {
    if (buffer[i] > max_val) max_val = buffer[i];
    if (buffer[i] < min_val) min_val = buffer[i];
  }

  // Set threshold at 50% of the signal range
  int16_t threshold = min_val + (max_val - min_val) / 2;

  // Count upward crossings
  int cycles = 0;
  for (int i = 1; i < buffer_size; i++) {  // start at 1, fixes buffer[-1] bug
    if (buffer[i] > threshold && buffer[i-1] <= threshold) {
      cycles++;
    }
  }

  return cycles;
}