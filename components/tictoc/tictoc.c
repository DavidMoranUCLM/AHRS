#include "tictoc.h"

#include <stdio.h>

#include "esp_log.h"
#include "string.h"

#define TICTOC_LIST_MAX 32
#define CYCLES_2_US (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ)

static const char* TAG = "TICTOC";
static tictoc_t tictoc_list[TICTOC_LIST_MAX] = {0};
static uint32_t tictoc_used = 0;

tictoc_t* tictoc_new(const char* name) {
  for (uint32_t i = 0; i < TICTOC_LIST_MAX; i++) {
    if (((tictoc_used >> i) & 0x1) == 0) {
      tictoc_used |= (1 << i);
      memset(&tictoc_list[i], 0, sizeof(tictoc_t));
      memcpy(tictoc_list[i].name, name, sizeof(tictoc_list[i].name) - 1);
      return &tictoc_list[i];
    }
  }
  return NULL;
}

void tictoc_free(tictoc_t* self) {
  for (uint32_t i = 0; i < TICTOC_LIST_MAX; i++) {
    if (self == &tictoc_list[i]) {
      tictoc_used &= ~(1 << i);
      break;
    }
  }
}

void tic(tictoc_t* self) {
  self->is_running = 1;
  self->start = esp_cpu_get_cycle_count();
}

void toc(tictoc_t* self) {
  uint32_t end = esp_cpu_get_cycle_count();

  if (!self->is_running) {
    return;
  }

  if (end < self->start) {
    return;
  }

  float elapsed = end - self->start;
  self->n++;
  if (self->n == 1) {
    self->stats.mean = elapsed;
    self->stats.M2 = 0;
    self->stats.min = elapsed;
    self->stats.max = elapsed;
  } else {
    float delta = elapsed - self->stats.mean;
    self->stats.mean += delta / (float)self->n;
    self->stats.M2 += delta * (elapsed - self->stats.mean);

    if (elapsed < self->stats.min) self->stats.min = elapsed;

    if (elapsed > self->stats.max) self->stats.max = elapsed;
  }
}

void tictoc_print(tictoc_t* self) {
  float var;
  if (self != NULL) {
    if (self->n == 0) {
      return;
    }

    var = self->stats.M2 / ((float)self->n - 1.f);
    var /= (float)(CYCLES_2_US * CYCLES_2_US);
    ESP_LOGI(TAG,
             "%16s: mean=%.2f , var=%.2f , min=%" PRIu32 ", max=%" PRIu32
             ", n=%" PRIu32,
             self->name, self->stats.mean / CYCLES_2_US, var,
             self->stats.min / CYCLES_2_US, self->stats.max / CYCLES_2_US,
             self->n);
  } else {
    for (uint8_t i = 0; i < TICTOC_LIST_MAX; i++) {
      if (((tictoc_used >> i) & 0x1) == 1) {
        self = &tictoc_list[i];
        var = self->stats.M2 / ((float)self->n - 1.f);
        var /= (float)(CYCLES_2_US * CYCLES_2_US);
        ESP_LOGI(TAG,
                 "%16s: mean=%.2f , var=%.2f , min=%" PRIu32 ", max=%" PRIu32
                 ", n=%" PRIu32,
                 self->name, self->stats.mean / CYCLES_2_US, var,
                 self->stats.min / CYCLES_2_US, self->stats.max / CYCLES_2_US,
                 self->n);
      }
    }
  }
}