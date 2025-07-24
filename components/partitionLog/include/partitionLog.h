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
  // // New EKF K matrix values (4 rows x 6 cols):
  // LOG_K11, LOG_K12, LOG_K13, LOG_K14, LOG_K15, LOG_K16,
  // LOG_K21, LOG_K22, LOG_K23, LOG_K24, LOG_K25, LOG_K26,
  // LOG_K31, LOG_K32, LOG_K33, LOG_K34, LOG_K35, LOG_K36,
  // LOG_K41, LOG_K42, LOG_K43, LOG_K44, LOG_K45, LOG_K46,
  // // New EKF S matrix values (6 rows x 6 cols):
  // LOG_S11,
  // LOG_S12,
  // LOG_S13,
  // LOG_S14,
  // LOG_S15,
  // LOG_S16,
  // LOG_S21,
  // LOG_S22,
  // LOG_S23,
  // LOG_S24,
  // LOG_S25,
  // LOG_S26,
  // LOG_S31,
  // LOG_S32,
  // LOG_S33,
  // LOG_S34,
  // LOG_S35,
  // LOG_S36,
  // LOG_S41,
  // LOG_S42,
  // LOG_S43,
  // LOG_S44,
  // LOG_S45,
  // LOG_S46,
  // LOG_S51,
  // LOG_S52,
  // LOG_S53,
  // LOG_S54,
  // LOG_S55,
  // LOG_S56,
  // LOG_S61,
  // LOG_S62,
  // LOG_S63,
  // LOG_S64,
  // LOG_S65,
  // LOG_S66,
  // LOG_P11,
  // LOG_P12,
  // LOG_P13,
  // LOG_P14,
  // LOG_P21,
  // LOG_P22,
  // LOG_P23,
  // LOG_P24,
  // LOG_P31,
  // LOG_P32,
  // LOG_P33,
  // LOG_P34,
  // LOG_P41,
  // LOG_P42,
  // LOG_P43,
  // LOG_P44,
  // LOH_H11,
  // LOG_H12,
  // LOG_H13,
  // LOG_H14,
  // LOH_H21,
  // LOG_H22,
  // LOG_H23,
  // LOG_H24,
  // LOH_H31,
  // LOG_H32,
  // LOG_H33,
  // LOG_H34,
  // LOH_H41,
  // LOG_H42,
  // LOG_H43,
  // LOG_H44,
  // LOH_H51,
  // LOG_H52,
  // LOG_H53,
  // LOG_H54,
  // LOH_H61,
  // LOG_H62,
  // LOG_H63,
  // LOG_H64,
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
  // // float K[4][6];   // <-- Added EKF K matrix values
  // float S[6][6];  // <-- New: EKF S matrix values (6x6)
  // float P[4][4];
  // float H[6][4];  // New: EKF H matrix values (6x4)
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
 * @return logPartition_t* Pointer to the new logPartition_t object, or NULL if
 * an error occurred.
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