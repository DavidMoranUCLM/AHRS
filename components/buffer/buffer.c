#include "buffer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "string.h"

static const char *TAG = "BUFFER";

/* Helper function: rounds up size to the next multiple of 4 */
static inline size_t padSize(size_t size) {
    return (size + 3) & ~0x03;
}

esp_err_t initBuffer(vectorBuffer_t *buffer, size_t itemSize, size_t maxItemNumber) {
    if (buffer == NULL) return ESP_FAIL;

    /* Use padded item size for ring buffer operations */
    size_t paddedItemSize = padSize(itemSize);
    size_t totalSize = paddedItemSize * maxItemNumber;

    /* Create a dynamic ring buffer.
       RINGBUF_TYPE_NOSPLIT ensures that items are never split across the buffer boundaries. */
    buffer->buffer = xRingbufferCreate(totalSize, RINGBUF_TYPE_NOSPLIT);
    if (buffer->buffer == NULL) {
        ESP_LOGE(TAG, "Dynamic ringbuffer creation failed (size %u)", (unsigned)totalSize);
        return ESP_FAIL;
    }

    buffer->vectorMaxNumber = maxItemNumber;
    buffer->vectorSize = paddedItemSize; // Use padded size internally

    buffer->mutex = xSemaphoreCreateMutex();
    configASSERT(buffer->mutex);

    /* --- Prime the ring buffer with a dummy push/pop --- */
    {
        uint8_t dummy[paddedItemSize];
        memset(dummy, 0, paddedItemSize);
        if (xRingbufferSend(buffer->buffer, dummy, paddedItemSize, 10) != pdTRUE) {
            ESP_LOGW(TAG, "Dummy push failed");
        } else {
            size_t dummySize = 0;
            void *dummyPtr = xRingbufferReceive(buffer->buffer, &dummySize, 10);
            if (dummyPtr != NULL) {
                vRingbufferReturnItem(buffer->buffer, dummyPtr);
            }
        }
    }
    /* ---------------------------------------------------- */

    /* Log ring buffer info */
    UBaseType_t freePtr, readPtr, writePtr, acquirePtr, itemsQueue;
    vRingbufferGetInfo(buffer->buffer, &freePtr, &readPtr, &writePtr, &acquirePtr, &itemsQueue);
    ESP_LOGD(TAG, "Buffer info. Ringbuffer ptr: %p", (void *)buffer->buffer);
    ESP_LOGD(TAG, "Ringbuffer Free Ptr: %u", freePtr);
    ESP_LOGD(TAG, "Ringbuffer Read Ptr: %u", readPtr);
    ESP_LOGD(TAG, "Ringbuffer Write Ptr: %u", writePtr);
    ESP_LOGD(TAG, "Ringbuffer Acquire Ptr: %u", acquirePtr);
    ESP_LOGD(TAG, "Ringbuffer Items Queue: %u", itemsQueue);

    return ESP_OK;
}

esp_err_t deinitBuffer(vectorBuffer_t *buffer) {
    if (buffer == NULL) return ESP_FAIL;
    vRingbufferDelete(buffer->buffer);
    vSemaphoreDelete(buffer->mutex);
    return ESP_OK;
}

esp_err_t pushItem(vectorBuffer_t *buffer, void *item, size_t itemLogicalSize) {
    configASSERT(item);
    configASSERT(buffer);
    configASSERT(buffer->buffer);

    if (xSemaphoreTake(buffer->mutex, 5) == pdTRUE) {
        size_t paddedSize = buffer->vectorSize;
        uint8_t temp[paddedSize];
        memset(temp, 0, paddedSize);
        memcpy(temp, item, itemLogicalSize);

        /* Log ring buffer info before sending */
        // UBaseType_t freePtr, readPtr, writePtr, acquirePtr, itemsQueue;
        // vRingbufferGetInfo(buffer->buffer, &freePtr, &readPtr, &writePtr, &acquirePtr, &itemsQueue);
        // ESP_LOGD(TAG, "In pushItem - Ringbuffer Free Ptr: %u", freePtr);
        // ESP_LOGD(TAG, "In pushItem - Ringbuffer Read Ptr: %u", readPtr);
        // ESP_LOGD(TAG, "In pushItem - Ringbuffer Write Ptr: %u", writePtr);
        // ESP_LOGD(TAG, "In pushItem - Ringbuffer Acquire Ptr: %u", acquirePtr);
        // ESP_LOGD(TAG, "In pushItem - Ringbuffer Items Queue: %u", itemsQueue);

        // ESP_LOGD(TAG, "In pushItem - Padded vector size: %u", paddedSize);
        // ESP_LOG_BUFFER_HEX_LEVEL(TAG, temp, paddedSize, ESP_LOG_DEBUG);

        if (xRingbufferSend(buffer->buffer, temp, paddedSize, 0) != pdTRUE) {
            xSemaphoreGive(buffer->mutex);
            return ESP_FAIL;
        }
        xSemaphoreGive(buffer->mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t pullItem(vectorBuffer_t *buffer, void *item, size_t itemLogicalSize) {
    configASSERT(item);
    configASSERT(buffer);
    configASSERT(buffer->buffer);

    size_t receivedSize = 0;
    if (xSemaphoreTake(buffer->mutex, 5) == pdTRUE) {
        void *itemCopy = xRingbufferReceive(buffer->buffer, &receivedSize, 0);
        if (itemCopy == NULL) {
            xSemaphoreGive(buffer->mutex);
            return ESP_ERR_TIMEOUT;
        }
        if (receivedSize != buffer->vectorSize) {
            vRingbufferReturnItem(buffer->buffer, itemCopy);
            xSemaphoreGive(buffer->mutex);
            return ESP_ERR_INVALID_RESPONSE;
        }
        memcpy(item, itemCopy, itemLogicalSize);
        vRingbufferReturnItem(buffer->buffer, itemCopy);
        xSemaphoreGive(buffer->mutex);
        return ESP_OK;
    }
    return ESP_FAIL;
}
