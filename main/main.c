#include <stdio.h>

#include "EKF.h"
#include "buffer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "i2cdev.h"
#include "logging.h"
#include "math.h"
#include "mpu9250.h"
#include "string.h"

#define CHECK(x)                       \
  do {                                 \
    esp_err_t __;                      \
    if ((__ = x) != ESP_OK) return __; \
  } while (0)

#define I2C_PORT 0
#define I2C_SDA 33
#define I2C_SCL 32

#define MEASURES_BUFFER_ITEM_NUMBER 20
#define RESULTS_BUFFER_ITEM_NUMBER 20

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
  vectorBuffer_t *buffer;
  mpu9250_dev_t *mpu9250;
  TickType_t taskPeriod;

} measurementsProducerCfg_t;

typedef struct {
  vectorBuffer_t *measuresBuffer;
  vectorBuffer_t *resultsBuffer;
  mpu9250_dev_t *mpu9250;
  EKF_ctx_t *ekf;
  TickType_t taskPeriod;

} measurementsProcesorCfg_t;

/**
 * Function declarations
 *
 */

esp_err_t app_init(mpu9250_dev_t *dev, EKF_ctx_t *ekf,
                   vectorBuffer_t *measuresBuffer,
                   vectorBuffer_t *resultsBuffer);

esp_err_t read_mpu9250_calibration(calibrationData_t *data);
esp_err_t apply_mpu9250_calibration(mpu9250_dev_t *dev,
                                    calibrationData_t *data);

void measurementsProducer(void *param);
void measurementsProcesor(void *param);
void logger();

/**
 * Function definitions
 *
 */

esp_err_t app_init(mpu9250_dev_t *dev, EKF_ctx_t *ekf,
                   vectorBuffer_t *measuresBuffer,
                   vectorBuffer_t *resultsBuffer) {
  const char *TAG = "APP_INIT";
  esp_log_level_set(TAG, ESP_LOG_DEBUG);

  configASSERT(dev);
  configASSERT(measuresBuffer);
  configASSERT(resultsBuffer);
  configASSERT(ekf);

  CHECK(i2cdev_init());
  ESP_LOGI(TAG, "I2C init success");

  CHECK(mpu9250_tests(dev, MPU6050_I2C_ADDRESS_LOW, I2C_SDA, I2C_SCL));

  CHECK(mpu9250_init_desc(dev, MPU6050_I2C_ADDRESS_LOW, I2C_PORT, I2C_SDA,
                          I2C_SCL));

  ESP_LOGI(TAG, "MPU9250 descriptor init success");

  CHECK(mpu9250_init(dev));
  ESP_LOGI(TAG, "MPU9250 init success");

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

  CHECK(initBuffer(measuresBuffer, sizeof(measureItem_t),
                   MEASURES_BUFFER_ITEM_NUMBER));
  ESP_LOGI(TAG, "Measurements buffer init success");
  CHECK(initBuffer(resultsBuffer, sizeof(resultItem_t),
                   RESULTS_BUFFER_ITEM_NUMBER));
  ESP_LOGI(TAG, "Results buffer init success");

  measures_t initMeasures;
  mpu9250_get_motion(dev, initMeasures.acc, initMeasures.velAng,
                     initMeasures.mag);
  ekfInit(ekf, &initMeasures);
  ESP_LOGI(TAG, "EKF init success");

  ESP_LOGI(TAG, "Free heap size: %lu KB", esp_get_free_heap_size() / 1000);

  vTaskDelay(200);

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

  TickType_t prevTick = 0, prevPrevTick = 0;
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

    do {
      err = pushItem(conf->buffer, &item, sizeof(item));
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Buffer push exception: %s", esp_err_to_name(err));
        vTaskDelay(1);
      }
    } while (err != ESP_OK);

    ESP_LOGD(TAG, "Item Pushed");
  }
}

void measurementsProcesor(void *param) {
  char *TAG = "PROCESOR_TASK";
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  ESP_LOGI(TAG, "Task Init");

  measurementsProcesorCfg_t *conf = param;

  TickType_t prevTick = 0, prevPrevTick = 0;
  measureItem_t measureItem;
  measures_t ekfMeasures;
  float mag_norm = 0;
  resultItem_t resultItem;
  esp_err_t err;

  while (1) {
    prevPrevTick = prevTick;
    do {
      err = pullItem(conf->measuresBuffer, &measureItem, sizeof(measureItem));
      if (err == ESP_ERR_TIMEOUT) break;
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Buffer pull exception: %s", esp_err_to_name(err));
        vTaskDelay(1);
        continue;
      }
      
      ekfMeasures.acc[0] = measureItem.acc.x;
      ekfMeasures.acc[1] = measureItem.acc.y;
      ekfMeasures.acc[2] = measureItem.acc.z;

      ekfMeasures.mag[0] = measureItem.mag.x;
      ekfMeasures.mag[1] = measureItem.mag.y;
      ekfMeasures.mag[2] = measureItem.mag.z;

      ekfMeasures.velAng[0] = measureItem.gyro.x-0.04;
      ekfMeasures.velAng[1] = measureItem.gyro.y+0.04;
      ekfMeasures.velAng[2] = measureItem.gyro.z+0.04;

      ekfStep(conf->ekf, &ekfMeasures, measureItem.measureTime);

      resultItem.measureTime = measureItem.measureTime;

      resultItem.quat[0] = gsl_quat_float_get(conf->ekf->q_current, 0);
      resultItem.quat[1] = gsl_quat_float_get(conf->ekf->q_current, 1);
      resultItem.quat[2] = gsl_quat_float_get(conf->ekf->q_current, 2);
      resultItem.quat[3] = gsl_quat_float_get(conf->ekf->q_current, 3);

      // Log accelerometer values
      ESP_LOGD(TAG, "Accelerometer values: X: %.2f, Y: %.2f, Z: %.2f",
               measureItem.acc.x, measureItem.acc.y, measureItem.acc.z);

      // Log magnetometer values
      ESP_LOGD(TAG, "Magnetometer values: X: %.2f, Y: %.2f, Z: %.2f",
               ekfMeasures.mag[0], ekfMeasures.mag[1], ekfMeasures.mag[2]);

      // Log angular velocity values
      ESP_LOGD(TAG, "Angular velocity values: X: %.2f, Y: %.2f, Z: %.2f",
        ekfMeasures.velAng[0], ekfMeasures.velAng[1], ekfMeasures.velAng[2]);
      ESP_LOGI(TAG, "t=%.4f, q0=%.2f, qx=%.2f, qy=%.2f, qz=%.2f",
               resultItem.measureTime, resultItem.quat[0], resultItem.quat[1],
               resultItem.quat[2], resultItem.quat[3]);
      //   do {
      //   err = pushItem(conf->resultsBuffer, &resultItem, sizeof(resultItem));
      //   if (err != ESP_OK) {
      //     ESP_LOGW(TAG, "Buffer push exception: %s",
      //              esp_err_to_name(err));
      //     vTaskDelay(1);
      //   }
      // } while (err != ESP_OK);

    } while (err != ESP_ERR_TIMEOUT);

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
  vectorBuffer_t measuresBuffer = {0};
  vectorBuffer_t resultsBuffer = {0};
  ESP_ERROR_CHECK(app_init(&dev, &ekf, &measuresBuffer, &resultsBuffer));

  measurementsProducerCfg_t producerCfg = {
      .buffer = &measuresBuffer,
      .mpu9250 = &dev,
      .taskPeriod = pdMS_TO_TICKS(50),
  };

  TaskHandle_t *producerTaskHandle;
  xTaskCreate(measurementsProducer, "Producer", configMINIMAL_STACK_SIZE * 10,
              &producerCfg, tskIDLE_PRIORITY, &producerTaskHandle);
  configASSERT(producerTaskHandle);
  vTaskSuspend(producerTaskHandle);

  measurementsProcesorCfg_t procesorCfg = {
      .ekf = &ekf,
      .measuresBuffer = &measuresBuffer,
      .resultsBuffer = &resultsBuffer,
      .mpu9250 = &dev,
      .taskPeriod = pdMS_TO_TICKS(500),
  };

  TaskHandle_t *procesorTaskHandle;
  xTaskCreate(measurementsProcesor, "Procesor", configMINIMAL_STACK_SIZE * 10,
              &procesorCfg, tskIDLE_PRIORITY, &procesorTaskHandle);
  configASSERT(procesorTaskHandle);
  vTaskSuspend(procesorTaskHandle);

  vTaskResume(producerTaskHandle);
  vTaskResume(procesorTaskHandle);

  vTaskSuspend(NULL);
}
