#ifndef TIC_TOC_H
#define TIC_TOC_H
#include "esp_cpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"

typedef struct {
  uint32_t start;
  uint32_t n;
  uint8_t is_running; 
  struct {
    float mean;
    float M2;
    uint32_t min;
    uint32_t max;
  } stats;
  char name[17];
} tictoc_t;

tictoc_t* tictoc_new(const char* name);
void tictoc_free(tictoc_t* self);
void tic(tictoc_t* self);
void toc(tictoc_t* self);
void tictoc_print(tictoc_t* self);
#endif // TIC_TOC_H