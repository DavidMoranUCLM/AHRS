#include <stdio.h>

#include "EKF.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "i2cdev.h"
#include "math.h"
#include "mpu9250.h"
#include "partitionLog.h"
#include "string.h"

#define CHECK(x)                       \
  do {                                 \
    esp_err_t __;                      \
    if ((__ = x) != ESP_OK) return __; \
  } while (0)

#define APP_INIT_BUTTON_PIN GPIO_NUM_23

#define I2C_PORT 0
#define I2C_SDA 33
#define I2C_SCL 32

#define MEASURES_BUFFER_ITEM_NUMBER 200
#define LOG_BUFFER_ITEM_NUMBER 200

#define LOG_ITEM_PERIOD_MS 100

#define US_TO_S ((float)(1.f / 1000000.f));

/**
 * Variable declarations
 *
 */

typedef struct __attribute__((__packed__)) {
  struct {
    float A[3][3];
    float b[3];
  } mag;
  struct {
    float bias[3];
    float scale[3];
  } acc;
  struct {
    float bias[3];
    float scale[3];
  } gyro;

} rawCalibrationData_t;

typedef struct {
  struct {
    float A[3][3];
    float b[3];
  } mag;
  struct {
    float bias[3];
    float scale[3];
  } acc;
  struct {
    float bias[3];
    float scale[3];
  } gyro;

} calibrationData_t;

typedef struct {
  mpu6050_acceleration_t acc;
  mpu6050_rotation_t gyro;
  ak8963_magnetometer_t mag;
  float measureTime;
} measureItem_t;

typedef struct {
  float quat[4];
  float measureTime;
} resultItem_t;

typedef struct {
  QueueHandle_t *queue;
  mpu9250_dev_t *mpu9250;
  TickType_t taskPeriod;

} measurementsProducerCfg_t;

typedef struct {
  QueueHandle_t *measuresQueue;
  QueueHandle_t *logQueue;
  mpu9250_dev_t *mpu9250;
  EKF_ctx_t *ekf;
  TickType_t taskPeriod;

} measurementsProcesorCfg_t;

typedef struct {
  QueueHandle_t *logQueue;
  logPartition_t *logPartition;
  TickType_t taskPeriod;
} loggerCfg_t;

/**
 * Function declarations
 *
 */

esp_err_t app_init(mpu9250_dev_t *dev, EKF_ctx_t *ekf,
                   QueueHandle_t **measuresQueue, QueueHandle_t **resultsQueue);

esp_err_t read_mpu9250_calibration(calibrationData_t *data);
esp_err_t apply_mpu9250_calibration(mpu9250_dev_t *dev,
                                    calibrationData_t *data);

void measurementsProducer(void *param);
void measurementsProcesor(void *param);
void logger(void *param);

/**
 * Function definitions
 *
 */

void lockUntilPress(void) {
  gpio_reset_pin(APP_INIT_BUTTON_PIN);
  gpio_set_direction(APP_INIT_BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(APP_INIT_BUTTON_PIN, GPIO_PULLUP_ONLY);
  fprintf(stdout, "Waiting for button press.\n");
  while (gpio_get_level(APP_INIT_BUTTON_PIN) == 1) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

esp_err_t app_init(mpu9250_dev_t *dev, EKF_ctx_t *ekf,
                   QueueHandle_t **measuresQueue, QueueHandle_t **logQueue) {
  const char *TAG = "APP_INIT";
  esp_log_level_set(TAG, ESP_LOG_DEBUG);

  configASSERT(dev);
  configASSERT(ekf);

  lockUntilPress();

  CHECK(i2cdev_init());
  ESP_LOGI(TAG, "I2C init success");

  CHECK(mpu9250_tests(dev, MPU6050_I2C_ADDRESS_LOW, I2C_SDA, I2C_SCL));

  CHECK(mpu9250_init_desc(dev, MPU6050_I2C_ADDRESS_LOW, I2C_PORT, I2C_SDA,
                          I2C_SCL));

  ESP_LOGI(TAG, "MPU9250 descriptor init success");

  CHECK(mpu9250_init(dev));
  ESP_LOGI(TAG, "MPU9250 init success");

  CHECK(mpu6050_set_dlpf_mode(dev, MPU6050_DLPF_2));

  calibrationData_t calibration = {0};
  CHECK(read_mpu9250_calibration(&calibration));
  for (uint8_t i = 0; i < 9; i++) {
    ESP_LOGD(TAG, "A Cal Data %u: %.2f", i, ((float *)calibration.mag.A)[i]);
  }

  for (uint8_t i = 0; i < 3; i++) {
    ESP_LOGD(TAG, "b Cal Data %u: %.2f", i, calibration.mag.b[i]);
  }

  CHECK(apply_mpu9250_calibration(dev, &calibration));
  ESP_LOGI(TAG, "MPU9250 calibration success");

  *measuresQueue =
      xQueueCreate(MEASURES_BUFFER_ITEM_NUMBER, sizeof(measureItem_t));
  CHECK(*measuresQueue);
  ESP_LOGI(TAG, "Measurements queue init success");

  *logQueue = xQueueCreate(LOG_BUFFER_ITEM_NUMBER, sizeof(logData_t));
  CHECK(*logQueue);
  ESP_LOGI(TAG, "Log queue init success");

  measures_t initMeasures;
  float mag[3];
  mpu9250_get_motion(dev, initMeasures.acc, initMeasures.velAng,
                     initMeasures.mag);
  ekfInit(ekf, &initMeasures);
  ESP_LOGI(TAG, "EKF init success");

  ESP_LOGI(TAG, "Free heap size: %lu KB", esp_get_free_heap_size() / 1000);

  return ESP_OK;
}

esp_err_t read_mpu9250_calibration(calibrationData_t *data) {
  configASSERT(data);
  const esp_partition_t *partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "calibration");
  configASSERT(partition);

  rawCalibrationData_t rawData;
  CHECK(esp_partition_read_raw(partition, 0, &rawData,
                               sizeof(rawCalibrationData_t)));

  memcpy(&data->mag, &rawData.mag, sizeof(rawData.mag));
  memcpy(&data->acc, &rawData.acc, sizeof(rawData.acc));
  memcpy(&data->gyro, &rawData.gyro, sizeof(rawData.gyro));

  return ESP_OK;
}

esp_err_t apply_mpu9250_calibration(mpu9250_dev_t *dev,
                                    calibrationData_t *data) {
  configASSERT(dev);
  configASSERT(data);

  CHECK(mpu9250_set_mag_cal(data->mag.A, data->mag.b));
  CHECK(mpu9250_set_accel_cal(data->acc.scale, data->acc.bias));
  CHECK(mpu9250_set_gyro_cal(data->gyro.scale, data->gyro.bias));

  return ESP_OK;
}

/**
 * Task definitions
 *
 */

void measurementsProducer(void *param) {
  char *TAG = "MEASURE_TASK";
  esp_log_level_set(TAG, ESP_LOG_INFO);
  ESP_LOGI(TAG, "Task Init");
  ESP_LOGD(TAG, "Debug Test");

  measurementsProducerCfg_t *conf = param;

  TickType_t prevTick = xTaskGetTickCount();
  TickType_t prevPrevTick = prevTick;
  measureItem_t item;

  esp_err_t err;
  while (1) {
    prevPrevTick = prevTick;
    do {
      if (xTaskDelayUntil(&prevTick, conf->taskPeriod) == pdFALSE) {
        ESP_LOGW(TAG, "Missed %ld ticks",
                 (prevTick - prevPrevTick) - conf->taskPeriod);
      }
      item.measureTime = (float)esp_timer_get_time() * US_TO_S;
      err = mpu9250_get_motion(conf->mpu9250, &item.acc, &item.gyro, &item.mag);
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "MPU9250 exception during reading: %s",
                 esp_err_to_name(err));
        vTaskDelay(1);
      }
    } while (err != ESP_OK);

    if (xQueueSend(*conf->queue, &item, portMAX_DELAY) != pdTRUE) {
      ESP_LOGW(TAG, "Buffer push exception: %s", esp_err_to_name(err));
      vTaskDelay(1);
    }

    ESP_LOGD(TAG, "Item Pushed");
  }
}

void measurementsProcesor(void *param) {
  char *TAG = "PROCESOR_TASK";
  esp_log_level_set(TAG, ESP_LOG_INFO);
  ESP_LOGI(TAG, "Task Init");

  measurementsProcesorCfg_t *conf = param;

  TickType_t prevTick = xTaskGetTickCount();
  TickType_t prevPrevTick = prevTick;
  measureItem_t measureItem;
  measures_t ekfMeasures;
  float mag_norm = 0;
  float mag[3] = {0};
  logData_t logItem = {0};
  esp_err_t err;

  while (1) {
    prevPrevTick = prevTick;

    if (xQueueReceive(*conf->measuresQueue, &measureItem, portMAX_DELAY) !=
        pdTRUE) {
      ESP_LOGW(TAG, "Buffer measures pull exception");
      vTaskDelay(1);
      continue;
    }

    ekfMeasures.acc[0] = measureItem.acc.x - 0.57;
    ekfMeasures.acc[1] = measureItem.acc.y + 0.9;
    ekfMeasures.acc[2] = measureItem.acc.z + 1.8;

    ekfMeasures.mag[0] = measureItem.mag.x;
    ekfMeasures.mag[1] = measureItem.mag.y;
    ekfMeasures.mag[2] = measureItem.mag.z;

    ekfMeasures.velAng[0] = measureItem.gyro.x + 0.045;
    ekfMeasures.velAng[1] = measureItem.gyro.y + 0.038;
    ekfMeasures.velAng[2] = measureItem.gyro.z - 0.045;

    ekfStep(conf->ekf, &ekfMeasures, measureItem.measureTime);

    if ((uint64_t)(measureItem.measureTime * 1000) % LOG_ITEM_PERIOD_MS <
        pdTICKS_TO_MS(conf->taskPeriod)) {
      logItem.timestamp = measureItem.measureTime;

      logItem.quat[0] = gsl_quat_float_get(conf->ekf->q_current, 0);
      logItem.quat[1] = gsl_quat_float_get(conf->ekf->q_current, 1);
      logItem.quat[2] = gsl_quat_float_get(conf->ekf->q_current, 2);
      logItem.quat[3] = gsl_quat_float_get(conf->ekf->q_current, 3);

      logItem.q_est[0] = gsl_quat_float_get(conf->ekf->q_est, 0);
      logItem.q_est[1] = gsl_quat_float_get(conf->ekf->q_est, 1);
      logItem.q_est[2] = gsl_quat_float_get(conf->ekf->q_est, 2);
      logItem.q_est[3] = gsl_quat_float_get(conf->ekf->q_est, 3);

      logItem.acc[0] = ekfMeasures.acc[0];
      logItem.acc[1] = ekfMeasures.acc[1];
      logItem.acc[2] = ekfMeasures.acc[2];
      logItem.gyro[0] = ekfMeasures.velAng[0];
      logItem.gyro[1] = ekfMeasures.velAng[1];
      logItem.gyro[2] = ekfMeasures.velAng[2];
      logItem.mag[0] = measureItem.mag.x;
      logItem.mag[1] = measureItem.mag.y;
      logItem.mag[2] = measureItem.mag.z;

      // for (uint8_t i = 0; i < conf->ekf->P_current->size1; i++) {
      //   for (uint8_t j = 0; j < conf->ekf->P_current->size2; j++) {
      //     logItem.P[i][j] = gsl_matrix_float_get(conf->ekf->P_current, i,
      //     j);
      //   }
      // }

      // for (uint8_t i = 0; i < conf->ekf->wk->S->size1; i++) {
      //   for (uint8_t j = 0; j < conf->ekf->wk->S->size2; j++) {
      //     logItem.S[i][j] = gsl_matrix_float_get(conf->ekf->wk->S, i, j);
      //   }
      // }
      // // New: Copy ekf->wk->H values into logItem.H (assumed dimensions
      // 6x4) for (uint8_t i = 0; i < conf->ekf->wk->H->size1; i++) {
      //   for (uint8_t j = 0; j < conf->ekf->wk->H->size2; j++) {
      //     logItem.H[i][j] = gsl_matrix_float_get(conf->ekf->wk->H, i, j);
      //   }
      // }

      float acc_norm = sqrtf(logItem.acc[0] * logItem.acc[0] +
                             logItem.acc[1] * logItem.acc[1] +
                             logItem.acc[2] * logItem.acc[2]);
      float mag_norm = sqrtf(logItem.mag[0] * logItem.mag[0] +
                             logItem.mag[1] * logItem.mag[1] +
                             logItem.mag[2] * logItem.mag[2]);
      // logItem.v[0] = gsl_vector_float_get(conf->ekf->wk->h, 0) -
      //                logItem.acc[0] / acc_norm;
      // logItem.v[1] = gsl_vector_float_get(conf->ekf->wk->h, 1) -
      //                logItem.acc[1] / acc_norm;
      // logItem.v[2] = gsl_vector_float_get(conf->ekf->wk->h, 2) -
      //                logItem.acc[2] / acc_norm;
      // logItem.v[3] = gsl_vector_float_get(conf->ekf->wk->h, 3) -
      //                logItem.mag[0] / mag_norm;
      // logItem.v[4] = gsl_vector_float_get(conf->ekf->wk->h, 4) -
      //                logItem.mag[1] / mag_norm;
      // logItem.v[5] = gsl_vector_float_get(conf->ekf->wk->h, 5) -
      //                logItem.mag[2] / mag_norm;
      logItem.v[0] = gsl_vector_float_get(conf->ekf->wk->h, 0);
      logItem.v[1] = gsl_vector_float_get(conf->ekf->wk->h, 1);
      logItem.v[2] = gsl_vector_float_get(conf->ekf->wk->h, 2);

      // Log accelerometer values
      ESP_LOGD(TAG, "Accelerometer values: X: %.2f, Y: %.2f, Z: %.2f",
               logItem.mag[0], logItem.mag[1], logItem.mag[2]);

      // Log angular velocity values
      ESP_LOGD(TAG, "Angular velocity values: X: %.2f, Y: %.2f, Z: %.2f",
               ekfMeasures.velAng[0], ekfMeasures.velAng[1],
               ekfMeasures.velAng[2]);
      ESP_LOGD(TAG, "t=%.4f, q0=%.2f, qx=%.2f, qy=%.2f, qz=%.2f",
               logItem.timestamp, logItem.quat[0], logItem.quat[1],
               logItem.quat[2], logItem.quat[3]);

      if (xQueueSend(*conf->logQueue, &logItem, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Buffer results push exception");
        vTaskDelay(1);
      }
    }

    if (xTaskDelayUntil(&prevTick, conf->taskPeriod) == pdFALSE) {
      ESP_LOGW(TAG, "Missed %ld ticks",
               (prevTick - prevPrevTick) - conf->taskPeriod);
    }
  }
}

void logger(void *param) {
  const char *TAG = "LOGGER_TASK";
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  ESP_LOGI(TAG, "Task Init");

  loggerCfg_t *conf = param;

  TickType_t prevTick = xTaskGetTickCount();
  TickType_t prevPrevTick = prevTick;
  logData_t logItem;
  esp_err_t err;

  while (1) {
    prevPrevTick = prevTick;

      if (xQueueSend(conf->logQueue, &logItem, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Buffer pull exception");
        vTaskDelay(1);
        continue;
      }

      conf->logPartition->data.timestamp = logItem.timestamp;
      conf->logPartition->data.quat[0] = logItem.quat[0];
      conf->logPartition->data.quat[1] = logItem.quat[1];
      conf->logPartition->data.quat[2] = logItem.quat[2];
      conf->logPartition->data.quat[3] = logItem.quat[3];
      conf->logPartition->data.q_est[0] = logItem.q_est[0];
      conf->logPartition->data.q_est[1] = logItem.q_est[1];
      conf->logPartition->data.q_est[2] = logItem.q_est[2];
      conf->logPartition->data.q_est[3] = logItem.q_est[3];
      conf->logPartition->data.acc[0] = logItem.acc[0];
      conf->logPartition->data.acc[1] = logItem.acc[1];
      conf->logPartition->data.acc[2] = logItem.acc[2];
      conf->logPartition->data.gyro[0] = logItem.gyro[0];
      conf->logPartition->data.gyro[1] = logItem.gyro[1];
      conf->logPartition->data.gyro[2] = logItem.gyro[2];
      conf->logPartition->data.mag[0] = logItem.mag[0];
      conf->logPartition->data.mag[1] = logItem.mag[1];
      conf->logPartition->data.mag[2] = logItem.mag[2];
      // for (uint8_t i = 0; i < 3; i++) {
      //   for (uint8_t j = 0; j < 3; j++) {
      //     conf->logPartition->data.P[i][j] = logItem.P[i][j];
      //   }
      // }

      // // NEW: Copy logItem.H values into the partition data
      // for (uint8_t i = 0; i < 6; i++) {
      //     for (uint8_t j = 0; j < 4; j++) {
      //       conf->logPartition->data.H[i][j] = logItem.H[i][j];
      //     }
      // }

      for (uint8_t i = 0; i < 3; i++) {
        conf->logPartition->data.v[i] = logItem.v[i];
      }

      do {
        err = logPartitionUpdate(conf->logPartition);
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "Log partition update exception: %s",
                   esp_err_to_name(err));
          vTaskDelay(1);
        }
      } while (err != ESP_OK);

    logPartitionUpdateHeader(conf->logPartition);
    ESP_LOGI(TAG, "Current Quat at time %.4f: %.4f, %.4f, %.4f, %.4f",
             logItem.timestamp, logItem.quat[0], logItem.quat[1],
             logItem.quat[2], logItem.quat[3]);
    ESP_LOGD(TAG, "Accelerometer values: X: %.2f, Y: %.2f, Z: %.2f",
             logItem.acc[0], logItem.acc[1], logItem.acc[2]);
    ESP_LOGD(TAG, "Angular velocity values: X: %.2f, Y: %.2f, Z: %.2f",
             logItem.gyro[0], logItem.gyro[1], logItem.gyro[2]);
    ESP_LOGD(TAG, "Magnetometer values: X: %.2f, Y: %.2f, Z: %.2f",
             logItem.mag[0], logItem.mag[1], logItem.mag[2]);
    ESP_LOGD(TAG, "Free heap size: %lu B", esp_get_free_heap_size());

    if (xTaskDelayUntil(&prevTick, conf->taskPeriod) == pdFALSE) {
      ESP_LOGW(TAG, "Missed %ld ticks",
               (prevTick - prevPrevTick) - conf->taskPeriod);
    }
  }
}

/**
 * Main
 *
 */

void app_main(void) {
  mpu9250_dev_t dev = {0};
  EKF_ctx_t ekf = {0};

  QueueHandle_t *measureQueue;
  QueueHandle_t *logQueue;

  ESP_ERROR_CHECK(app_init(&dev, &ekf, &measureQueue, &logQueue));

  loggerCfg_t loggerCfg = {
      .logQueue = &logQueue,
      .logPartition = logPartitionNew(),
      .taskPeriod = pdMS_TO_TICKS(250),
  };

  configASSERT(loggerCfg.logPartition);
  TaskHandle_t *loggerTaskHandle;
  xTaskCreatePinnedToCore(logger, "Logger", configMINIMAL_STACK_SIZE * 10,
                          &loggerCfg, tskIDLE_PRIORITY, &loggerTaskHandle, 1);
  configASSERT(loggerTaskHandle);
  vTaskSuspend(loggerTaskHandle);

  measurementsProducerCfg_t producerCfg = {
      .queue = &measureQueue,
      .mpu9250 = &dev,
      .taskPeriod = pdMS_TO_TICKS(50),
  };

  TaskHandle_t *producerTaskHandle;
  xTaskCreatePinnedToCore(measurementsProducer, "Producer",
                          configMINIMAL_STACK_SIZE * 10, &producerCfg, 0,
                          &producerTaskHandle, 0);
  configASSERT(producerTaskHandle);
  vTaskSuspend(producerTaskHandle);

  measurementsProcesorCfg_t procesorCfg = {
      .ekf = &ekf,
      .measuresQueue = &measureQueue,
      .logQueue = &logQueue,
      .mpu9250 = &dev,
      .taskPeriod = pdMS_TO_TICKS(200),
  };

  TaskHandle_t *procesorTaskHandle;
  xTaskCreatePinnedToCore(measurementsProcesor, "Procesor",
                          configMINIMAL_STACK_SIZE * 10, &procesorCfg,
                          tskIDLE_PRIORITY, &procesorTaskHandle, 1);
  configASSERT(procesorTaskHandle);
  vTaskSuspend(procesorTaskHandle);

  vTaskResume(producerTaskHandle);
  vTaskResume(procesorTaskHandle);
  vTaskResume(loggerTaskHandle);

  vTaskSuspend(NULL);
}
