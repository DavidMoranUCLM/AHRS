#ifndef PARTITION_LOG_H
#define PARTITION_LOG_H

#include "esp_partition.h"

/**
 * @brief Enum for data layout in the log.
 */
typedef enum {
  LOG_TIMESTAMP_S,
  LOG_ACC_X,
  LOG_ACC_Y,
  LOG_ACC_Z,
  LOG_GYRO_X,
  LOG_GYRO_Y,
  LOG_GYRO_Z,
  LOG_MAG_X,
  LOG_MAG_Y,
  LOG_MAG_Z,
  LOG_QUAT_W,
  LOG_QUAT_X,
  LOG_QUAT_Y,
  LOG_QUAT_Z,
  // New q_est values:
  LOG_QEST_W,
  LOG_QEST_X,
  LOG_QEST_Y,
  LOG_QEST_Z,
  // New ekf->h values:
  LOG_V0,
  LOG_V1,
  LOG_V2,
  LOG_V3,
  LOG_V4,
  LOG_V5,
  LOG_P11, LOG_P12, LOG_P13, LOG_P14,
  LOG_P21, LOG_P22, LOG_P23, LOG_P24,
  LOG_P31, LOG_P32, LOG_P33, LOG_P34,
  LOG_P41, LOG_P42, LOG_P43, LOG_P44,
  LOG_MAX_ELEM,
} dataLayout_t;

/**
 * @brief Struct for log data.
 */
typedef struct __attribute__((__packed__)) {
  float timestamp;
  float acc[3];
  float gyro[3];
  float mag[3];
  float quat[4];
  float q_est[4];  // <-- Added EKF q_est values
  float v[6];      // EKF h values (renamed from h)
  float P[4][4];
} logData_t;


typedef struct __attribute__((__packed__)) {
  uint32_t nData;
  char varNames[LOG_MAX_ELEM][10];
} logHeader_t;


/**
 * @brief Struct for log partition.
 */
typedef struct {
  esp_partition_t *partition;
  logHeader_t header;
  uint32_t head;
  uint32_t headerSize;
  logData_t data;
} logPartition_t;

/**
 * @brief Create a new log partition object.
 *
 * @return logPartition_t* Pointer to the new logPartition_t object, or NULL if an error occurred.
 */
logPartition_t *logPartitionNew(void);

/**
 * @brief Update the log partition with new data.
 *
 * @param logPartition Pointer to the logPartition_t object.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t logPartitionUpdate(logPartition_t *logPartition);

/**
 * @brief Update the header of the partition.
 *
 * @param logPartition Pointer to the logPartition_t object.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t logPartitionUpdateHeader(logPartition_t *logPartition);

/**
 * @brief Deinitialize the log partition object.
 *
 * @param logPartition Pointer to the logPartition_t object.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t logPartitionDeInit(logPartition_t *logPartition);

#endif