#include "ahrs.h"

#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gsl/gsl_matrix_float.h"
#include "gsl/gsl_vector_float.h"
#include "gsl_quaternion_float.h"
#include "math_utils.h"
#include "string.h"

static const char *TAG = "AHRS";

BaseType_t xQueueForceSendToBack(QueueHandle_t queue, void *item,
                                 size_t item_size) {
  char buf[item_size];
  if (uxQueueSpacesAvailable(queue) == 0) {
    xQueueReceive(queue, &buf,
                  0);  // Remove the oldest item if the queue is full
  }
  BaseType_t res = xQueueSendToBack(queue, item, 0);
  return res;
}

void correct_velocity_from_gps(ahrs_state_t *state,
                               ahrs_gps_queue_item_t *gps_item,
                               ahrs_gps_queue_item_t *gps_item_prev) {
  if (state == NULL || gps_item == NULL || gps_item_prev == NULL) {
    ESP_LOGE(TAG, "Invalid arguments to correct_velocity_from_gps");
    return;
  }

  // Extract latitude and longitude in degrees
  float lat, lon, lat_prev, lon_prev;
  wgs84_get_full_range(&gps_item->coor, &lat, &lon);
  wgs84_get_full_range(&gps_item_prev->coor, &lat_prev, &lon_prev);

  // Convert to radians
  float lat_rad = lat * M_PI / 180.0f;
  float lon_rad = lon * M_PI / 180.0f;
  float lat_prev_rad = lat_prev * M_PI / 180.0f;
  float lon_prev_rad = lon_prev * M_PI / 180.0f;

  // Compute time delta in seconds
  float delta_time_s =
      (float)(gps_item->time_us - gps_item_prev->time_us) / 1e6f;
  if (delta_time_s <= 0) {
    ESP_LOGE(TAG, "Invalid delta time: %f seconds", delta_time_s);
    memset(&gps_item->coor, 0,
           sizeof(gps_item->coor));  // Reset GPS item coordinates
    return;
  }

  // Compute deltas
  float delta_phi = lat_rad - lat_prev_rad;
  float delta_lambda = lon_rad - lon_prev_rad;
  float delta_h = gps_item->coor.height_AMSL - gps_item_prev->coor.height_AMSL;

  // Precompute trigonometric values
  float s_phi = sinf(lat_rad);
  float s_phi_2 = s_phi * s_phi;
  float c_phi = cosf(lat_rad);

  // Precompute constants
  float t1 = 1.f / delta_time_s;
  float a1 = 1 - 0.00669437999f * s_phi_2;
  float a2 = sqrtf(a1);
  float a3 = t1 / (a2 * a2 * a2);

  float b1 = a3 * 6335439.327f;
  float b2 = a2 * 6378137.0f * c_phi;

  // Compute velocity vector
  gsl_vector_float *vel = gsl_vector_float_alloc(3);
  gsl_vector_float_set(vel, 0, b1 * delta_phi);
  gsl_vector_float_set(vel, 1, b2 * delta_lambda);
  gsl_vector_float_set(vel, 2, delta_h * t1);

  // Jacobian of velocity with respect to delta
  gsl_matrix_float *jac_delta = gsl_matrix_float_calloc(3, 3);
  gsl_matrix_float_set(jac_delta, 0, 0, b1);
  gsl_matrix_float_set(jac_delta, 1, 1, b2);
  gsl_matrix_float_set(jac_delta, 2, 2, t1);

  // Jacobian with respect to latitude (phi)
  gsl_vector_float *jac_phi = gsl_vector_float_alloc(3);
  float denom1 = delta_time_s *
                 powf(0.003347189995f * cosf(2 * lat_rad) + 0.99665281f, 2.5f);
  float numer1 = 6335439.327f * delta_phi * sinf(2 * lat_rad);
  float jac_phi_0 = denom1 != 0.0f ? numer1 / denom1 : 0.0f;

  float numer2 = -3.0e-13f * delta_lambda * s_phi *
                 (278521.0f * s_phi_2 + 2.111813109e19f);
  float jac_phi_1 = numer2 * a3;

  gsl_vector_float_set(jac_phi, 0, jac_phi_0);
  gsl_vector_float_set(jac_phi, 1, jac_phi_1);
  gsl_vector_float_set(jac_phi, 2, 0.0f);

  // Covariance propagation
  gsl_matrix_float *delta_covar = gsl_matrix_float_calloc(3, 3);
  gsl_matrix_float_set(
      delta_covar, 0, 0,
      gps_item->coor.variance[0] + gps_item_prev->coor.variance[0]);
  gsl_matrix_float_set(
      delta_covar, 1, 1,
      gps_item->coor.variance[1] + gps_item_prev->coor.variance[1]);
  gsl_matrix_float_set(
      delta_covar, 2, 2,
      gps_item->coor.variance[2] + gps_item_prev->coor.variance[2]);

  gsl_matrix_float *tmp = gsl_matrix_float_calloc(3, 3);
  gsl_blas_sgemm(CblasNoTrans, CblasNoTrans, 1.0f, jac_delta, delta_covar, 0.0f,
                 tmp);
  gsl_blas_sgemm(CblasNoTrans, CblasTrans, 1.0f, tmp, jac_delta, 1.0f,
                 delta_covar);

  gsl_matrix_float *phi_covar = gsl_matrix_float_calloc(3, 3);
  gsl_blas_sger(gps_item->coor.variance[0], jac_phi, jac_phi, phi_covar);

  gsl_matrix_float_add(delta_covar, phi_covar);

  // Fuse velocity estimates
  normal_dist_intersection(state->vel, vel, state->vel, state->vel_covar,
                           delta_covar, state->vel_covar);

  // Cleanup
  gsl_vector_float_free(vel);
  gsl_matrix_float_free(jac_delta);
  gsl_vector_float_free(jac_phi);
  gsl_matrix_float_free(phi_covar);
  gsl_matrix_float_free(tmp);
  gsl_matrix_float_free(delta_covar);
}

void quat_rotvec_jac(gsl_vector_float *v, gsl_matrix_float *jac) {
  float v1, v2, v3;
  v1 = gsl_vector_float_get(v, 0);
  v2 = gsl_vector_float_get(v, 1);
  v3 = gsl_vector_float_get(v, 2);

  gsl_matrix_float_set(jac, 0, 0, v1);
  gsl_matrix_float_set(jac, 0, 1, 0);
  gsl_matrix_float_set(jac, 0, 2, v3);
  gsl_matrix_float_set(jac, 0, 3, -v2);

  gsl_matrix_float_set(jac, 1, 0, v2);
  gsl_matrix_float_set(jac, 1, 1, -v3);
  gsl_matrix_float_set(jac, 1, 2, 0);
  gsl_matrix_float_set(jac, 1, 3, v1);

  gsl_matrix_float_set(jac, 2, 0, v3);
  gsl_matrix_float_set(jac, 2, 1, v2);
  gsl_matrix_float_set(jac, 2, 2, -v1);
  gsl_matrix_float_set(jac, 2, 3, 0);
}

void integrate_body_acceleration(ahrs_state_t *state, float acc[3],
                                 float acc_var[3], float quat[4],
                                 float q_covar[4][4], uint64_t delta_us) {
  if (state == NULL || acc == NULL || quat == NULL || q_covar == NULL) {
    ESP_LOGE(TAG, "Invalid arguments to integrate_body_acceleration");
    return;
  }

  float delta_s = (float)delta_us / 1e6f;  // Convert microseconds to seconds
  float delta_s2 = delta_s * delta_s;

  // Quaternion setup and rotate acceleration to horizontal frame
  gsl_quat_float *q = gsl_quat_float_calloc();
  gsl_quat_float_set(q, 0, quat[0]);
  gsl_quat_float_set(q, 1, quat[1]);
  gsl_quat_float_set(q, 2, quat[2]);
  gsl_quat_float_set(q, 3, quat[3]);
  gsl_quat_float_conjugate(q);

  gsl_vector_float *acc_vec = gsl_vector_float_calloc(3);
  gsl_vector_float_set(acc_vec, 0, acc[0]);
  gsl_vector_float_set(acc_vec, 1, acc[1]);
  gsl_vector_float_set(acc_vec, 2, acc[2]);

  gsl_vector_float *acc_hor = gsl_vector_float_calloc(3);
  gsl_quat_float_rotvec(q, acc_vec, acc_hor);

  gsl_quat_float_free(q);
  gsl_vector_float_free(acc_vec);

  // Jacobian of quaternion rotation
  gsl_matrix_float *jac = gsl_matrix_float_calloc(3, 4);
  quat_rotvec_jac(acc_hor, jac);

  // Acceleration covariance in horizontal frame
  gsl_matrix_float *acc_hor_covar = gsl_matrix_float_calloc(3, 3);
  gsl_matrix_float_set(acc_hor_covar, 0, 0, acc_var[0]);
  gsl_matrix_float_set(acc_hor_covar, 1, 1, acc_var[1]);
  gsl_matrix_float_set(acc_hor_covar, 2, 2, acc_var[2]);

  gsl_matrix_float *q_covar_m = gsl_matrix_float_alloc(4, 4);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      gsl_matrix_float_set(q_covar_m, i, j, q_covar[i][j]);
    }
  }

  gsl_matrix_float *tmp = gsl_matrix_float_calloc(3, 4);
  gsl_blas_sgemm(CblasNoTrans, CblasNoTrans, 1.0f, jac, q_covar_m, 0.0f, tmp);
  gsl_blas_sgemm(CblasNoTrans, CblasTrans, 1.0f, tmp, jac, 1.f, acc_hor_covar);
  gsl_matrix_float_free(tmp);
  gsl_matrix_float_free(jac);

  // Velocity update and covariance
  gsl_vector_float *vel_delta = gsl_vector_float_alloc(3);
  gsl_vector_float_memcpy(vel_delta, acc_hor);
  gsl_vector_float_scale(vel_delta, delta_s);
  gsl_vector_float_add(state->vel, vel_delta);

  gsl_matrix_float *vel_jac_acc = gsl_matrix_float_alloc(3, 3);
  gsl_matrix_float_set_identity(vel_jac_acc);
  gsl_matrix_float_scale(vel_jac_acc, delta_s);

  gsl_matrix_float *tmp_vel = gsl_matrix_float_calloc(3, 3);
  gsl_blas_sgemm(CblasNoTrans, CblasNoTrans, 1.0f, vel_jac_acc, acc_hor_covar,
                 0.0f, tmp_vel);
  gsl_blas_sgemm(CblasNoTrans, CblasTrans, 1.0f, tmp_vel, vel_jac_acc, 1.f,
                 state->vel_covar);

  gsl_matrix_float_free(tmp_vel);
  gsl_matrix_float_free(vel_jac_acc);
  gsl_vector_float_free(vel_delta);

  // Position update and covariance
  gsl_vector_float *pos_delta0 = gsl_vector_float_alloc(3);
  gsl_vector_float_memcpy(pos_delta0, state->vel);
  gsl_vector_float_scale(pos_delta0, delta_s);

  gsl_vector_float *pos_delta1 = gsl_vector_float_alloc(3);
  gsl_vector_float_memcpy(pos_delta1, acc_hor);
  gsl_vector_float_scale(pos_delta1, 0.5f * delta_s2);

  gsl_vector_float_add(state->pos, pos_delta0);
  gsl_vector_float_add(state->pos, pos_delta1);

  gsl_vector_float_free(pos_delta0);
  gsl_vector_float_free(pos_delta1);

  gsl_matrix_float *pos_jac_acc = gsl_matrix_float_alloc(3, 3);
  gsl_matrix_float_set_identity(pos_jac_acc);
  gsl_matrix_float_scale(pos_jac_acc, delta_s2);

  gsl_matrix_float *tmp_pos = gsl_matrix_float_calloc(3, 3);
  gsl_blas_sgemm(CblasNoTrans, CblasNoTrans, 1.0f, pos_jac_acc, acc_hor_covar,
                 0.0f, tmp_pos);
  gsl_blas_sgemm(CblasNoTrans, CblasTrans, 1.0f, tmp_pos, pos_jac_acc, 1.f,
                 state->pos_covar);

  gsl_matrix_float_free(tmp_pos);
  gsl_matrix_float_free(pos_jac_acc);

  gsl_matrix_float_free(acc_hor_covar);
  gsl_vector_float_free(acc_hor);
}

void calculate_aproximate_coordinates(ahrs_state_t *state) {
  if (state == NULL) {
    ESP_LOGE(TAG, "Invalid arguments to calculate_aproximate_coordinates");
    return;
  }

  float jac_delta[3][3] = {0};
  float jac_coor[3][3] = {0};

  wgs84_ENU_delta_update_jac_coor(
      state->coor, gsl_vector_float_get(state->pos, 0),
      gsl_vector_float_get(state->pos, 1), gsl_vector_float_get(state->pos, 2),
      jac_coor);

  wgs84_ENU_delta_update_jac_delta(
      state->coor, gsl_vector_float_get(state->pos, 0),
      gsl_vector_float_get(state->pos, 1), gsl_vector_float_get(state->pos, 2),
      jac_delta);

  gsl_matrix_float *covar = gsl_matrix_float_calloc(3, 3);
  gsl_matrix_float *covar_delta = gsl_matrix_float_calloc(3, 3);
  gsl_matrix_float *jac_delta_m = gsl_matrix_float_alloc(3, 3);
  gsl_matrix_float *jac_coor_m = gsl_matrix_float_alloc(3, 3);
  gsl_matrix_float *covar_coor = NULL;

  if (!covar || !covar_delta || !jac_delta_m || !jac_coor_m) {
    ESP_LOGE(TAG,
             "Memory allocation failed in calculate_aproximate_coordinates");
    goto cleanup;
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      gsl_matrix_float_set(jac_delta_m, i, j, jac_delta[i][j]);
      gsl_matrix_float_set(jac_coor_m, i, j, jac_coor[i][j]);
    }
  }

  gsl_blas_sgemm(CblasNoTrans, CblasNoTrans, 1.0f, jac_delta_m,
                 state->pos_covar, 0.0f, covar);
  gsl_blas_sgemm(CblasNoTrans, CblasTrans, 1.0f, covar, jac_delta_m, 1.f,
                 covar_delta);

  covar_coor = gsl_matrix_float_calloc(3, 3);
  if (!covar_coor) {
    ESP_LOGE(TAG, "Memory allocation failed for covar_coor");
    goto cleanup;
  }

  gsl_matrix_float_set(covar_coor, 0, 0, state->coor->variance[0]);
  gsl_matrix_float_set(covar_coor, 1, 1, state->coor->variance[1]);
  gsl_matrix_float_set(covar_coor, 2, 2, state->coor->variance[2]);

  gsl_blas_sgemm(CblasNoTrans, CblasNoTrans, 1.0f, jac_coor_m, covar_coor, 0.0f,
                 covar);
  gsl_blas_sgemm(CblasNoTrans, CblasTrans, 1.0f, covar, jac_coor_m, 1.f,
                 covar_delta);

  state->coor->variance[0] = gsl_matrix_float_get(covar_delta, 0, 0);
  state->coor->variance[1] = gsl_matrix_float_get(covar_delta, 1, 1);
  state->coor->variance[2] = gsl_matrix_float_get(covar_delta, 2, 2);

  wgs84_ENU_delta_update(state->coor, state->pos->data[0], state->pos->data[1],
                         state->pos->data[2]);

cleanup:
  if (covar) gsl_matrix_float_free(covar);
  if (covar_delta) gsl_matrix_float_free(covar_delta);
  if (jac_delta_m) gsl_matrix_float_free(jac_delta_m);
  if (jac_coor_m) gsl_matrix_float_free(jac_coor_m);
  if (covar_coor) gsl_matrix_float_free(covar_coor);
}

void combine_coordinates(ahrs_state_t *state, wgs84_coordinates_t *coor) {
  if (state == NULL || coor == NULL) {
    ESP_LOGE(TAG, "Invalid arguments to combine_coordinates");
    return;
  }

  wgs84_degree2decimal(coor);

  gsl_vector_float *x1 = gsl_vector_float_alloc(3);
  gsl_vector_float *x2 = gsl_vector_float_alloc(3);
  gsl_vector_float *x3 = gsl_vector_float_calloc(3);
  gsl_matrix_float *covar1 = gsl_matrix_float_calloc(3, 3);
  gsl_matrix_float *covar2 = gsl_matrix_float_calloc(3, 3);
  gsl_matrix_float *covar3 = gsl_matrix_float_calloc(3, 3);

  if (x1 == NULL || x2 == NULL || x3 == NULL || covar1 == NULL ||
      covar2 == NULL || covar3 == NULL) {
    ESP_LOGE(TAG, "Memory allocation failed");
    return;
  }
  gsl_vector_float_set(x1, 0, coor->lat.dec);
  gsl_vector_float_set(x1, 1, coor->lon.dec);
  gsl_vector_float_set(x1, 2, coor->height_AMSL);

  gsl_vector_float_set(x2, 0, state->coor->lat.dec);
  gsl_vector_float_set(x2, 1, state->coor->lon.dec);
  gsl_vector_float_set(x2, 2, state->coor->height_AMSL);

  gsl_matrix_float_set(covar1, 0, 0, coor->variance[0]);
  gsl_matrix_float_set(covar1, 1, 1, coor->variance[1]);
  gsl_matrix_float_set(covar1, 2, 2, coor->variance[2]);

  gsl_matrix_float_set(covar2, 0, 0, state->coor->variance[0]);
  gsl_matrix_float_set(covar2, 1, 1, state->coor->variance[1]);
  gsl_matrix_float_set(covar2, 2, 2, state->coor->variance[2]);

  normal_dist_intersection(x1, x2, x3, covar1, covar2, covar3);

  wgs84_set_dec_coor(gsl_vector_float_get(x3, 1), gsl_vector_float_get(x3, 0),
                     state->coor);
  state->coor->height_AMSL = gsl_vector_float_get(x3, 2);

  state->coor->variance[0] = gsl_matrix_float_get(covar3, 0, 0);
  state->coor->variance[1] = gsl_matrix_float_get(covar3, 1, 1);
  state->coor->variance[2] = gsl_matrix_float_get(covar3, 2, 2);

  wgs84_decimal2degree(state->coor);

  gsl_vector_float_free(x1);
  gsl_vector_float_free(x2);
  gsl_vector_float_free(x3);
  gsl_matrix_float_free(covar1);
  gsl_matrix_float_free(covar2);
  gsl_matrix_float_free(covar3);
}

void ahrs_imu_task_t(const ahrs_cfg_t *cfg) {
  ESP_LOGI(TAG, "AHRS IMU Task started");
  ahrs_imu_callback_data_t imu_data;

  void (*imu_callback)(ahrs_imu_callback_data_t *data) = cfg->imu_callback;
  if (imu_callback == NULL) {
    ESP_LOGE(TAG, "IMU callback is not set");
    return;
  }

  uint32_t task_period_ms = cfg->imu_task_period_ms;
  if (task_period_ms == 0) {
    ESP_LOGE(TAG, "IMU callback period is set to 0");
    return;
  }

  QueueHandle_t imu_queue = cfg->imu_queue;
  if (imu_queue == NULL) {
    ESP_LOGE(TAG, "IMU queue is not set");
    return;
  }

  uint64_t time_us;

  ahrs_imu_queue_item_t queue_item;

  while (1) {
    imu_callback(&imu_data);
    time_us = esp_timer_get_time();

    for (uint16_t n = 0; n < imu_data.sample_number; n++) {
      queue_item.acc[0] = imu_data.acc[n][0];
      queue_item.acc[1] = imu_data.acc[n][1];
      queue_item.acc[2] = imu_data.acc[n][2];
      queue_item.gyro[0] = imu_data.gyro[n][0];
      queue_item.gyro[1] = imu_data.gyro[n][1];
      queue_item.gyro[2] = imu_data.gyro[n][2];
      queue_item.mag[0] = imu_data.mag[n][0];
      queue_item.mag[1] = imu_data.mag[n][1];
      queue_item.mag[2] = imu_data.mag[n][2];
      queue_item.time_us = time_us + imu_data.delta_time_us[n];
      time_us = queue_item.time_us;
      if (xQueueSendToBack(imu_queue, &queue_item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to send IMU data to queue");
      }
      if (xQueueForceSendToBack(cfg->imu_history_queue, &queue_item,
                                sizeof(queue_item)) != pdTRUE) {
        // ESP_LOGW(TAG, "Failed to force send IMU data to history queue");
      }
    }

    if (xTaskDelay(pdMS_TO_TICKS(task_period_ms)) == pdFALSE) {
      ESP_LOGW(TAG, "IMU task deadline missed");
    }
  }
}

void ahrs_gps_task_t(const ahrs_cfg_t *cfg) {
  ESP_LOGI(TAG, "AHRS GPS Task started");
  void (*gps_callback)(wgs84_coordinates_t *gps_data) = cfg->gps_callback;
  if (gps_callback == NULL) {
    ESP_LOGE(TAG, "GPS callback is not set");
    return;
  }

  uint32_t task_period_ms = cfg->gps_task_period_ms;
  if (task_period_ms == 0) {
    ESP_LOGE(TAG, "GPS callback period is set to 0");
    return;
  }

  QueueHandle_t gps_queue = cfg->gps_queue;
  if (gps_queue == NULL) {
    ESP_LOGE(TAG, "GPS queue is not set");
    return;
  }

  ahrs_gps_queue_item_t queue_item;

  while (1) {
    queue_item.time_us = esp_timer_get_time();
    gps_callback(&queue_item.coor);

    if (xQueueSendToBack(gps_queue, &queue_item, 0) != pdTRUE) {
      ESP_LOGW(TAG, "Failed to send GPS data to queue");
    }

    if (xTaskDelay(pdMS_TO_TICKS(task_period_ms)) == pdFALSE) {
      ESP_LOGW(TAG, "GPS task deadline missed");
    }
    if (xQueueForceSendToBack(cfg->gps_history_queue, &queue_item,
                              sizeof(queue_item)) != pdTRUE) {
      // ESP_LOGW(TAG, "Failed to force send IMU data to history queue");
    }
  }
}

void ahrs_attitude_task_t(const ahrs_cfg_t *cfg) {
  ESP_LOGI(TAG, "AHRS Attitude Task started");
  ahrs_attitude_callback_data_t attitude_data;

  void (*attitude_callback)(ahrs_attitude_callback_data_t *data) =
      cfg->attitude_callback;
  if (attitude_callback == NULL) {
    ESP_LOGE(TAG, "Attitude callback is not set");
    return;
  }

  uint32_t task_period_ms = cfg->attitude_task_period_ms;
  if (task_period_ms == 0) {
    ESP_LOGE(TAG, "Attitude callback period is set to 0");
    return;
  }

  QueueHandle_t attitude_queue = cfg->attitude_queue;
  if (attitude_queue == NULL) {
    ESP_LOGE(TAG, "Attitude queue is not set");
    return;
  }

  ahrs_attitude_queue_item_t attitude_queue_item;
  ahrs_imu_queue_item_t imu_queue_item;

  while (1) {
    while (1) {
      BaseType_t res = xQueueReceive(cfg->imu_queue, &imu_queue_item, 0);
      if (res == pdFALSE) {
        break;  // No data in the queue
      }

      attitude_data.time_us =
          imu_queue_item.time_us;  // Use the time from the IMU data

      attitude_data.acc[0] = imu_queue_item.acc[0];
      attitude_data.acc[1] = imu_queue_item.acc[1];
      attitude_data.acc[2] = imu_queue_item.acc[2];
      attitude_data.gyro[0] = imu_queue_item.gyro[0];
      attitude_data.gyro[1] = imu_queue_item.gyro[1];
      attitude_data.gyro[2] = imu_queue_item.gyro[2];
      attitude_data.mag[0] = imu_queue_item.mag[0];
      attitude_data.mag[1] = imu_queue_item.mag[1];
      attitude_data.mag[2] = imu_queue_item.mag[2];

      attitude_callback(&attitude_data);

      memcpy(attitude_queue_item.quat, attitude_data.quat,
             sizeof(attitude_data.quat));
      memcpy(attitude_queue_item.acc, attitude_data.acc,
             sizeof(attitude_data.acc));
      memcpy(attitude_queue_item.acc_var, attitude_data.acc_var,
             sizeof(attitude_data.acc_var));
      memcpy(attitude_queue_item.quat_var, attitude_data.quat_var,
             sizeof(attitude_data.quat_var));

      if (xQueueSendToBack(attitude_queue, &attitude_queue_item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to send attitude data to queue");
      }
      if (xQueueForceSendToBack(cfg->attitude_history_queue, &queue_item,
                                sizeof(queue_item)) != pdTRUE) {
        // ESP_LOGW(TAG, "Failed to force send IMU data to history queue");
      }
    }

    if (xTaskDelay(pdMS_TO_TICKS(task_period_ms)) == pdFALSE) {
      ESP_LOGW(TAG, "Attitude task deadline missed");
    }
  }
}

void ahrs_update_task_t(const ahrs_cfg_t *cfg) {
  ESP_LOGI(TAG, "AHRS Update Task started");
  ahrs_state_t *state = cfg->state;

  uint32_t task_period_ms = cfg->update_task_period_ms;
  if (task_period_ms == 0) {
    ESP_LOGE(TAG, "Attitude callback period is set to 0");
    return;
  }

  ahrs_attitude_queue_item_t attitude_item;
  ahrs_gps_queue_item_t gps_item, gps_item_prev;
  ahrs_state_queue_item_t state_item;

  QueueHandle_t state_queue = cfg->state_queue;
  if (state_queue == NULL) {
    ESP_LOGE(TAG, "State queue is not set");
    return;
  }
  QueueHandle_t gps_queue = cfg->gps_queue;
  if (gps_queue == NULL) {
    ESP_LOGE(TAG, "GPS queue is not set");
    return;
  }
  QueueHandle_t attitude_queue = cfg->attitude_queue;
  if (attitude_queue == NULL) {
    ESP_LOGE(TAG, "Attitude queue is not set");
    return;
  }

  BaseType_t attitude_res, gps_res;

  while (1) {
    while (1) {
      attitude_res = xQueuePeek(attitude_queue, &attitude_item, 0);
      gps_res = xQueuePeek(gps_queue, &gps_item, 0);

      if (attitude_res == pdFALSE && gps_res == pdFALSE) {
        // No data available, break the loop
        break;
      }

      if (attitude_res == pdTRUE && gps_res == pdTRUE) {
        if (attitude_item.time_us < gps_item.time_us) {
          // If attitude data is older than GPS data, we process attitude data
          xQueueReceive(attitude_queue, &attitude_item, 0);
          gps_res = pdFALSE;
        } else if (gps_item.time_us < attitude_item.time_us) {
          // If GPS data is older than attitude data, we process GPS data
          xQueueReceive(gps_queue, &gps_item, 0);
          attitude_res = pdFALSE;
        }
      } else {
        xQueueReceive(gps_queue, &gps_item, 0);
        attitude_res = pdFALSE;  // We prioritize GPS data
      }

      if (gps_res == pdTRUE) {
        calculate_aproximate_coordinates(state);
        combine_coordinates(state, &gps_item.coor);
        correct_velocity_from_gps(state, &gps_item, &gps_item_prev);
        memcpy(&gps_item_prev, &gps_item, sizeof(gps_item_prev));
      }

      if (attitude_res == pdTRUE) {
        uint64_t delta_us = attitude_item.time_us - state->time_us;
        integrate_body_acceleration(state, attitude_item.acc,
                                    attitude_item.acc_var, attitude_item.quat,
                                    attitude_item.quat_var, delta_us);
        state->time_us = attitude_item.time_us;
      }

      // Prepare state item for the queue
      memcpy(&state_item.coor, state->coor, sizeof(state_item.coor));
      state_item.time_us = state->time_us;

      for (int i = 0; i < 3; i++) {
        state_item.pos[i] = gsl_vector_float_get(state->pos, i);
        state_item.vel[i] = gsl_vector_float_get(state->vel, i);
        for (int j = 0; j < 3; j++) {
          state_item.vel_covar[i][j] =
              gsl_matrix_float_get(state->vel_covar, i, j);
          state_item.pos_covar[i][j] =
              gsl_matrix_float_get(state->pos_covar, i, j);
        }
      }

      if (xQueueSendToBack(state_queue, &state_item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to send state data to queue");
      }
      if (xQueueForceSendToBack(cfg->state_history_queue, &state_item,
                                sizeof(state_item)) != pdTRUE) {
        // ESP_LOGW(TAG, "Failed to force send state data to history queue");
      }
    };

    calculate_aproximate_coordinates(state);

    if (xTaskDelay(pdMS_TO_TICKS(task_period_ms)) == pdFALSE) {
      ESP_LOGW(TAG, "Update task deadline missed");
    }
  }
}

/**
 * @brief Initializes the AHRS system with the provided configuration.
 *
 * This function sets up the necessary queues, allocates memory for the AHRS
 * state, and validates the configuration parameters. If any errors occur during
 * initialization, the function cleans up allocated resources and returns an
 * error code.
 *
 * @param cfg Pointer to the AHRS configuration structure. Must not be NULL.
 *            The structure should contain valid callback functions and task
 * periods.
 *
 * @return AHRS_OK on successful initialization.
 *         AHRS_ERR_INVALID_ARG if the configuration is invalid.
 *         AHRS_ERR if memory allocation or queue creation fails.
 */
ahrs_err_t ahrs_init(ahrs_cfg_t *cfg) {
  if (cfg == NULL) {
    ESP_LOGE(TAG, "AHRS configuration is NULL");
    return AHRS_ERR_INVALID_ARG;
  }
  if (cfg->imu_callback == NULL) {
    ESP_LOGE(TAG, "IMU callback is not set");
    return AHRS_ERR_INVALID_ARG;
  }
  if (cfg->gps_callback == NULL) {
    ESP_LOGE(TAG, "GPS callback is not set");
    return AHRS_ERR_INVALID_ARG;
  }
  if (cfg->attitude_callback == NULL) {
    ESP_LOGE(TAG, "Attitude callback is not set");
    return AHRS_ERR_INVALID_ARG;
  }
  if (cfg->imu_task_period_ms == 0) {
    ESP_LOGE(TAG, "IMU task period is set to 0");
    return AHRS_ERR_INVALID_ARG;
  }
  if (cfg->gps_task_period_ms == 0) {
    ESP_LOGE(TAG, "GPS task period is set to 0");
    return AHRS_ERR_INVALID_ARG;
  }
  if (cfg->attitude_task_period_ms == 0) {
    ESP_LOGE(TAG, "Attitude task period is set to 0");
    return AHRS_ERR_INVALID_ARG;
  }
  if (cfg->update_task_period_ms == 0) {
    ESP_LOGE(TAG, "Update task period is set to 0");
    return AHRS_ERR_INVALID_ARG;
  }

  cfg->imu_queue =
      xQueueCreate(AHRS_IMU_QUEUE_SIZE, sizeof(ahrs_imu_queue_item_t));
  if (cfg->imu_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create IMU queue");
    goto cleanup_err;
  }

  cfg->gps_queue =
      xQueueCreate(AHRS_GPS_QUEUE_SIZE, sizeof(ahrs_gps_queue_item_t));
  if (cfg->gps_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create GPS queue");
    goto cleanup_err;
  }

  cfg->attitude_queue = xQueueCreate(AHRS_ATTITUDE_QUEUE_SIZE,
                                     sizeof(ahrs_attitude_queue_item_t));
  if (cfg->attitude_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create Attitude queue");
    goto cleanup_err;
  }

  cfg->state_queue =
      xQueueCreate(AHRS_STATE_QUEUE_SIZE, sizeof(ahrs_state_queue_item_t));
  if (cfg->state_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create State queue");
    goto cleanup_err;
  }

  cfg->imu_history_queue =
      xQueueCreate(AHRS_IMU_HISTORY_QUEUE_SIZE, sizeof(ahrs_imu_queue_item_t));
  if (cfg->imu_history_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create IMU history queue");
    goto cleanup_err;
  }
  cfg->gps_history_queue =
      xQueueCreate(AHRS_GPS_HISTORY_QUEUE_SIZE, sizeof(ahrs_gps_queue_item_t));
  if (cfg->gps_history_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create GPS history queue");
    goto cleanup_err;
  }

  cfg->attitude_history_queue = xQueueCreate(
      AHRS_ATTITUDE_HISTORY_QUEUE_SIZE, sizeof(ahrs_attitude_queue_item_t));
  if (cfg->attitude_history_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create Attitude history queue");
    goto cleanup_err;
  }

  cfg->state_history_queue = xQueueCreate(AHRS_STATE_HISTORY_QUEUE_SIZE,
                                          sizeof(ahrs_state_queue_item_t));
  if (cfg->state_history_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create State history queue");
    goto cleanup_err;
  }

  // Initialize the state
  cfg->state = calloc(1, sizeof(ahrs_state_t));
  if (cfg->state == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for AHRS state");
    goto cleanup_err;
  }

  cfg->state->coor = calloc(1, sizeof(wgs84_coordinates_t));
  if (cfg->state->coor == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for AHRS coordinates");
    free(cfg->state);
    goto cleanup_err;
  }

  cfg->state->pos = gsl_vector_float_calloc(3);
  if (cfg->state->pos == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for AHRS position vector");
    goto cleanup_err;
  }
  cfg->state->vel = gsl_vector_float_calloc(3);
  if (cfg->state->vel == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for AHRS velocity vector");
    goto cleanup_err;
  }
  cfg->state->vel_covar = gsl_matrix_float_calloc(3, 3);
  if (cfg->state->vel_covar == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for AHRS velocity covariance");
    goto cleanup_err;
  }
  cfg->state->pos_covar = gsl_matrix_float_calloc(3, 3);
  if (cfg->state->pos_covar == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for AHRS position covariance");
    goto cleanup_err;
  }
  cfg->state->time_us = 0;

  return AHRS_OK;
cleanup_err:
  ahrs_deinit(cfg);
  return AHRS_ERR;
}

/**
 * @brief Deinitializes the AHRS system and frees allocated resources.
 *
 * This function cleans up all queues and memory allocated during the
 * initialization of the AHRS system. It ensures that all pointers are set to
 * NULL after freeing their associated resources to prevent dangling references.
 *
 * @param cfg Pointer to the AHRS configuration structure. Must not be NULL.
 *            The structure should contain valid pointers to queues and state
 * data.
 *
 * @return AHRS_OK on successful deinitialization.
 *         AHRS_ERR_INVALID_ARG if the configuration pointer is NULL.
 */
ahrs_err_t ahrs_deinit(ahrs_cfg_t *cfg) {
  if (cfg == NULL) {
    ESP_LOGE(TAG, "AHRS configuration is NULL");
    return AHRS_ERR_INVALID_ARG;
  } else {
    if (cfg->imu_queue) {
      vQueueDelete(cfg->imu_queue);
      cfg->imu_queue = NULL;
    }
    if (cfg->gps_queue) {
      vQueueDelete(cfg->gps_queue);
      cfg->gps_queue = NULL;
    }
    if (cfg->attitude_queue) {
      vQueueDelete(cfg->attitude_queue);
      cfg->attitude_queue = NULL;
    }
    if (cfg->state_queue) {
      vQueueDelete(cfg->state_queue);
      cfg->state_queue = NULL;
    }
    if (cfg->imu_history_queue) {
      vQueueDelete(cfg->imu_history_queue);
      cfg->imu_history_queue = NULL;
    }
    if (cfg->gps_history_queue) {
      vQueueDelete(cfg->gps_history_queue);
      cfg->gps_history_queue = NULL;
    }
    if (cfg->attitude_history_queue) {
      vQueueDelete(cfg->attitude_history_queue);
      cfg->attitude_history_queue = NULL;
    }
    if (cfg->state_history_queue) {
      vQueueDelete(cfg->state_history_queue);
      cfg->state_history_queue = NULL;
    }
    if (cfg->state) {
      if (cfg->state->coor) {
        gsl_vector_float_free(cfg->state->coor);
        cfg->state->coor = NULL;
      }
      if (cfg->state->pos) {
        gsl_vector_float_free(cfg->state->pos);
        cfg->state->pos = NULL;
      }
      if (cfg->state->vel) {
        gsl_vector_float_free(cfg->state->vel);
        cfg->state->vel = NULL;
      }
      if (cfg->state->vel_covar) {
        gsl_matrix_float_free(cfg->state->vel_covar);
        cfg->state->vel_covar = NULL;
      }
      if (cfg->state->pos_covar) {
        gsl_matrix_float_free(cfg->state->pos_covar);
        cfg->state->pos_covar = NULL;
      }
      free(cfg->state);
      cfg->state = NULL;
    }
  }
  return AHRS_OK;
}