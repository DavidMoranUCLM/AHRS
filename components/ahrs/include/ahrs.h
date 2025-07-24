#include "WGS84.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gsl/gsl_matrix_float.h"
#include "gsl/gsl_vector_float.h"

#define AHRS_MAX_QUEUE_SIZE 100
#define AHRS_IMU_QUEUE_SIZE 10
#define AHRS_GPS_QUEUE_SIZE 10
#define AHRS_ATTITUDE_QUEUE_SIZE 10
#define AHRS_STATE_QUEUE_SIZE 10
#define AHRS_IMU_HISTORY_QUEUE_SIZE 100
#define AHRS_GPS_HISTORY_QUEUE_SIZE 100
#define AHRS_ATTITUDE_HISTORY_QUEUE_SIZE 100
#define AHRS_STATE_HISTORY_QUEUE_SIZE 100

typedef enum {
  AHRS_OK = 0,
  AHRS_ERR = -1,
  AHRS_ERR_NOT_INITIALIZED = -2,
  AHRS_ERR_INVALID_ARG = -3,
  AHRS_ERR_NO_MEM = -4,
} ahrs_err_t;

typedef struct {
  wgs84_coordinates_t *coor;
  gsl_vector_float *pos;
  gsl_vector_float *vel;
  gsl_matrix_float *vel_covar;
  gsl_matrix_float *pos_covar;
  uint64_t time_us;
} ahrs_state_t;


typedef struct{
  wgs84_coordinates_t coor;
  float pos[3];
  float vel[3];
  float vel_covar[3][3];
  float pos_covar[3][3];
  uint64_t time_us;
}ahrs_state_queue_item_t;


typedef struct {
  float (*acc)[3];
  float (*gyro)[3];
  float (*mag)[3];
  uint64_t *delta_time_us;
  uint16_t sample_number;
} ahrs_imu_callback_data_t;

typedef struct {
  float acc[3];
  float gyro[3];
  float mag[3];
  float quat[4];
  float acc_var[3];
  float quat_var[4][4];
  uint64_t time_us;
} ahrs_attitude_callback_data_t;

typedef struct {
  wgs84_coordinates_t *gps_data;
} ahrs_gps_callback_data_t;

typedef struct ahrs_attitude_queue_item_s {
  float quat[4];
  float acc[3];
  float acc_var[3];
  float quat_var[4][4];
  uint64_t time_us;
} ahrs_attitude_queue_item_t;

typedef struct ahrs_gps_queue_item_s {
  wgs84_coordinates_t coor;
  uint64_t time_us;
} ahrs_gps_queue_item_t;

typedef struct ahrs_imu_queue_item_s {
  float acc[3];
  float gyro[3];
  float mag[3];
  uint64_t time_us;
} ahrs_imu_queue_item_t;

typedef struct {
  void (*imu_callback)(ahrs_imu_callback_data_t *data);
  void (*attitude_callback)(ahrs_attitude_callback_data_t *data);
  void (*gps_callback)(wgs84_coordinates_t *gps_data);

  uint32_t imu_task_period_ms;
  uint32_t attitude_task_period_ms;
  uint32_t gps_task_period_ms;
  uint32_t update_task_period_ms;

  QueueHandle_t imu_queue;
  QueueHandle_t gps_queue;
  QueueHandle_t attitude_queue;
  QueueHandle_t state_queue;

  QueueHandle_t imu_history_queue;
  QueueHandle_t gps_history_queue;
  QueueHandle_t attitude_history_queue;
  QueueHandle_t state_history_queue;

  TaskHandle_t imu_task_handle;
  TaskHandle_t gps_task_handle;
  TaskHandle_t attitude_task_handle;

  ahrs_state_t *state;
} ahrs_cfg_t;

void ahrs_imu_task_t(const ahrs_cfg_t *cfg);
void ahrs_gps_task_t(const ahrs_cfg_t *cfg);
void ahrs_attitude_task_t(const ahrs_cfg_t *cfg);
void ahrs_update_task_t(const ahrs_cfg_t *cfg);

ahrs_err_t ahrs_init(ahrs_cfg_t *cfg);
ahrs_err_t ahrs_deinit(ahrs_cfg_t *cfg);