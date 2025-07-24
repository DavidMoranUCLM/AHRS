#ifndef BUFFER_H
#define BUFFER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

// Structure for the dynamic ring buffer.
typedef struct {
    RingbufHandle_t buffer;  // Dynamic ring buffer handle
    size_t vectorMaxNumber;  // Maximum number of items in the ring buffer
    size_t vectorSize;       // Padded size for each item (used internally)
    SemaphoreHandle_t mutex; // Mutex for thread-safe access
} vectorBuffer_t;

/**
 * @brief Initializes a dynamic ring buffer.
 *
 * @param buffer Pointer to a vectorBuffer_t structure.
 * @param itemSize Logical size of one item (in bytes).
 * @param maxItemNumber Maximum number of items.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t initBuffer(vectorBuffer_t *buffer, size_t itemSize, size_t maxItemNumber);

/**
 * @brief Deinitializes the ring buffer.
 *
 * @param buffer Pointer to a vectorBuffer_t structure.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t deinitBuffer(vectorBuffer_t *buffer);

/**
 * @brief Pushes an item into the ring buffer.
 *
 * @param buffer Pointer to a vectorBuffer_t structure.
 * @param item Pointer to the item data.
 * @param itemLogicalSize Logical size of the item (in bytes).
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t pushItem(vectorBuffer_t *buffer, void *item, size_t itemLogicalSize);

/**
 * @brief Pulls an item from the ring buffer.
 *
 * @param buffer Pointer to a vectorBuffer_t structure.
 * @param item Pointer to a buffer where the item data will be copied.
 * @param itemLogicalSize Logical size of the item (in bytes).
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t pullItem(vectorBuffer_t *buffer, void *item, size_t itemLogicalSize);

#endif // BUFFER_H
