#include "partitionLog.h"

#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "math.h"
#include "string.h"

#define LOG_PARTITION_TAG "LOG_PARTITION"
#define LOG_PARTITION_NAME "storage"

// Updated static array with variable names based on the enum order in partitionLog.h
static const char *varNames[LOG_MAX_ELEM] = {
  "time_s",
  "acc_x",
  "acc_y",
  "acc_z",
  "gyro_x",
  "gyro_y",
  "gyro_z",
  "mag_x",
  "mag_y",
  "mag_z",
  "quat_w",
  "quat_x",
  "quat_y",
  "quat_z",
  "q_est_w",
  "q_est_x",
  "q_est_y",
  "q_est_z",
  "v0", "v1", "v2", "v3", "v4", "v5",  // New ekf->h names
  "P11", "P12", "P13", "P14",
  "P21", "P22", "P23", "P24",
  "P31", "P32", "P33", "P34",
  "P41", "P42", "P43", "P44"
};

static size_t getHeaderSize(const logPartition_t *logPartition) {
  if (logPartition->partition->erase_size > sizeof(logPartition->header)) {
    return logPartition->partition->erase_size;
  } else {
    float size = (float)sizeof(logPartition->header) /
                 (float)logPartition->partition->erase_size;
    return ceil(size) * logPartition->partition->erase_size;
  }
}

/**
 * @brief Create a new log partition object.
 *
 * This function allocates memory for a new logPartition_t object and
 * initializes it. It finds the partition with the name defined by
 * LOG_PARTITION_NAME.
 *
 * @return logPartition_t* Pointer to the new logPartition_t object, or NULL if
 * an error occurred.
 */
logPartition_t *logPartitionNew(void) {
  esp_err_t err;
  logPartition_t *logPartition = calloc(1,sizeof(logPartition_t));
  if (logPartition == NULL) {
    ESP_LOGE(LOG_PARTITION_TAG, "Error allocating memory");
    return NULL;
  }
  logPartition->partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, LOG_PARTITION_NAME);
  if (logPartition->partition == NULL) {
    ESP_LOGE(LOG_PARTITION_TAG, "Partition not found");
    return ESP_FAIL;
  }
  ESP_LOGI(LOG_PARTITION_TAG, "Partition found");

  logPartition->headerSize = getHeaderSize(logPartition);
  
  logPartition->header.nData = 0;
  // Copy each variable name into header.varNames
  for (int i = 0; i < LOG_MAX_ELEM; i++) {
    strncpy(logPartition->header.varNames[i], varNames[i], sizeof(logPartition->header.varNames[i]) - 1);
    logPartition->header.varNames[i][sizeof(logPartition->header.varNames[i]) - 1] = '\0';
  }

  logPartition->head = logPartition->headerSize;
  // Use partition->erase_size logic to compute total erase size:
  uint32_t totalBlocks =
      logPartition->partition->size / logPartition->partition->erase_size;
  uint32_t totalEraseSize = totalBlocks * logPartition->partition->erase_size;
  esp_err_t ret =
      esp_partition_erase_range(logPartition->partition, 0, totalEraseSize);
  if (ret != ESP_OK) {
    ESP_LOGE(LOG_PARTITION_TAG,
             "Failed to erase entire partition using erase_size: %s",
             esp_err_to_name(ret));
    free(logPartition);
    return NULL;
  }
  err = esp_partition_write(logPartition->partition, 0, &logPartition->header,
                            sizeof(logPartition->header));
  if (err != ESP_OK) {
    ESP_LOGE(LOG_PARTITION_TAG, "Error writing to partition: %s",
             esp_err_to_name(err));
    free(logPartition);
    return NULL;
  }
  return logPartition;
}

/**
 * @brief Update the log partition with new data.
 *
 * This function writes the current number of data entries and the new data to
 * the partition. If the partition is full, it resets the head to the beginning.
 *
 * @param logPartition Pointer to the logPartition_t object.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t logPartitionUpdate(logPartition_t *logPartition) {
  if (logPartition->head > logPartition->partition->size) {
    ESP_LOGE(LOG_PARTITION_TAG, "Partition full");
    return ESP_ERR_FLASH_BASE;
  }
  esp_err_t ret =
      esp_partition_write(logPartition->partition, logPartition->head,
                          &logPartition->data, sizeof(logPartition->data));
  if (ret != ESP_OK) {
    ESP_LOGE(LOG_PARTITION_TAG, "Error writing log data to partition: %s",
             esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGD(LOG_PARTITION_TAG, "Data written to partition");
  logPartition->head += sizeof(logPartition->data);
  logPartition->header.nData++;
  return ESP_OK;
}

esp_err_t logPartitionUpdateHeader(logPartition_t *logPartition) {
  // Erase header using the erase block size if necessary
  esp_err_t ret = esp_partition_erase_range(logPartition->partition, 0,
                                            logPartition->headerSize);
  if (ret != ESP_OK) {
    ESP_LOGE(LOG_PARTITION_TAG, "Failed to erase header: %s",
             esp_err_to_name(ret));
    return ret;
  }
  ret = esp_partition_write(logPartition->partition, 0, &logPartition->header,
                            sizeof(logPartition->header));
  if (ret != ESP_OK) {
    ESP_LOGE(LOG_PARTITION_TAG, "Error writing header to partition: %s",
             esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGD(LOG_PARTITION_TAG, "Header written to partition");
  return ESP_OK;
}

/**
 * @brief Deinitialize the log partition object.
 *
 * This function frees the memory allocated for the logPartition_t object.
 *
 * @param logPartition Pointer to the logPartition_t object.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t logPartitionDeInit(logPartition_t *logPartition) {
  free(logPartition);
  return ESP_OK;
}