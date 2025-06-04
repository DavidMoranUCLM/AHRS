#include "gsl_quaternion_float.h"

#include <math.h>

#include "gsl/gsl_eigen.h"
#include "gsl/gsl_matrix_float.h"
#include "gsl/gsl_vector_float.h"

#define ROT_MAT_SIZE 3
#define QUAT_SIZE 4
#define QUAT_MATRIX_SIZE 4
#define NORMALIZE_DELTA (nextafterf(1, 2) - 1)
#define QUAT_MINIMUM_AXIS_NORM 1e-3f
#define FLOAT_ERROR 1e-5f

/**
 * Private definitions
 **/

/**
 * Private functions declarations
 */
static float fastSqrtf(float);
static gsl_matrix *rotMatGetK(gsl_matrix_float *pRotMat);

/**
 * Public functions definitions
 */
gsl_quat_float *gsl_quat_float_alloc(void) {
  return (gsl_quat_float *)gsl_vector_float_alloc(QUAT_SIZE);
}

gsl_quat_float *gsl_quat_float_calloc(void) {
  return (gsl_quat_float *)gsl_vector_float_calloc(QUAT_SIZE);
}

void gsl_quat_float_free(gsl_quat_float *pQ) {
  gsl_vector_float_free((gsl_vector_float *)pQ);
}

float gsl_quat_float_norm(gsl_quat_float *q) {
  float norm = 0;

  for (size_t i = 0; i < q->size; i++) {
    norm += q->data[i] * q->data[i];
  }

  if (fabs(norm - 1) > NORMALIZE_DELTA) {
    return sqrtf(norm);
  }
  return fastSqrtf(norm - 1);
}

void gsl_quat_float_set(gsl_quat_float *pQ, uint8_t i, float value) {
  gsl_vector_float_set(pQ, i, value);
}

float gsl_quat_float_get(gsl_quat_float *pQ, uint8_t i) {
  return gsl_vector_float_get(pQ, i);
}

int gsl_quat_float_get_imaginary(gsl_quat_float *pQ, gsl_vector_float *pV) {
  gsl_vector_float_set(pV, 0, gsl_quat_float_get(pQ, 1));
  gsl_vector_float_set(pV, 1, gsl_quat_float_get(pQ, 2));
  gsl_vector_float_set(pV, 2, gsl_quat_float_get(pQ, 3));

  return 0;
}

void gsl_quat_float_normalize(gsl_quat_float *q) {
  float norm = gsl_quat_float_norm(q);

  gsl_vector_float_scale(q, 1 / norm);
}

void gsl_quat_float_conjugate(gsl_quat_float *pQ) {
  pQ->data[0] = pQ->data[0];
  pQ->data[1] = -pQ->data[1];
  pQ->data[2] = -pQ->data[2];
  pQ->data[3] = -pQ->data[3];
}

void gsl_quat_float_copy(const gsl_quat_float *qSrc, gsl_quat_float *qDst) {
  for (uint8_t i = 0; i < QUAT_SIZE; i++) {
    qDst->data[i] = qSrc->data[i];
  }
}

void gsl_quat_float_product(gsl_quat_float *q1, const gsl_quat_float *q2) {
  float q3[4];

  q3[0] = q1->data[0] * q2->data[0] - q1->data[1] * q2->data[1] -
          q1->data[2] * q2->data[2] - q1->data[3] * q2->data[3];
  q3[1] = q1->data[0] * q2->data[1] + q1->data[1] * q2->data[0] +
          q1->data[2] * q2->data[3] - q1->data[3] * q2->data[2];
  q3[2] = q1->data[0] * q2->data[2] - q1->data[1] * q2->data[3] +
          q1->data[2] * q2->data[0] + q1->data[3] * q2->data[1];
  q3[3] = q1->data[0] * q2->data[3] + q1->data[1] * q2->data[2] -
          q1->data[2] * q2->data[1] + q1->data[3] * q2->data[0];
  for (uint8_t i = 0; i < QUAT_SIZE; i++) {
    gsl_quat_float_set(q1, i, q3[i]);
  }
}

int gsl_quat_float_inv(gsl_quat_float *pQ) {
  if (gsl_quat_float_norm(pQ) - 1 > FLOAT_ERROR) {
    return -1;
  }

  gsl_quat_float_conjugate(pQ);

  return 0;
}

int gsl_quat_float_fromAxis(gsl_vector_float *pAxis, const float angleRad,
                            gsl_quat_float *pQ) {
  if (pAxis->size != 3) {
    return -1;
  }

  if (pQ == NULL) {
    return -1;
  }

  float norm = 0;
  for (int i = 0; i < 3; i++) {
    norm += pAxis->data[i] * pAxis->data[i];
  }

  norm = sqrtf(norm);

  float sinHalfAngle = sinf(angleRad / 2);

  pQ->data[0] = cosf(angleRad / 2);
  pQ->data[1] = pAxis->data[0] * sinHalfAngle / norm;
  pQ->data[2] = pAxis->data[1] * sinHalfAngle / norm;
  pQ->data[3] = pAxis->data[2] * sinHalfAngle / norm;

  gsl_quat_float_normalize(pQ);
  return 0;
}

int gsl_quat_float_fromVector(gsl_vector_float *pVector, gsl_quat_float *pQ) {
  if (pVector->size != 3) {
    return -1;
  }

  if (pQ == NULL) {
    return -1;
  }

  gsl_quat_float_set(pQ, 0, 0);
  for (uint8_t i = 0; i < pVector->size; i++) {
    pQ->data[i + 1] = pVector->data[i];
  }

  return 0;
}


int gsl_quat_float_fromEuler(float roll, float pitch, float yaw, gsl_quat_float *pQ){
  if (pQ == NULL) {
    return -1;
  }

  float cy = cosf(yaw * 0.5f);
  float sy = sinf(yaw * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);

  pQ->data[0] = cr * cp * cy + sr * sp * sy;
  pQ->data[1] = sr * cp * cy - cr * sp * sy;
  pQ->data[2] = cr * sp * cy + sr * cp * sy;
  pQ->data[3] = cr * cp * sy - sr * sp * cy;

  return 0;
}


int gsl_quat_float_fromRotMatrix(gsl_matrix_float *pRotMat,
                                 gsl_quat_float *pQ) {
  if (pRotMat->size1 != ROT_MAT_SIZE || pRotMat->size2 != ROT_MAT_SIZE) {
    return -1;
  }
  if (pQ == NULL) {
    return -1;
  }

  gsl_matrix *K = rotMatGetK(pRotMat);
  gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc(4);
  gsl_vector *eigenValues = gsl_vector_alloc(4);
  gsl_matrix *eigenVectors = gsl_matrix_alloc(4, 4);

  gsl_eigen_symmv(K, eigenValues, eigenVectors, w);
  gsl_eigen_symmv_free(w);
  gsl_matrix_free(K);

  gsl_eigen_symmv_sort(eigenValues, eigenVectors, GSL_EIGEN_SORT_VAL_DESC);

  gsl_quat_float_set(pQ, 1, (float)gsl_matrix_get(eigenVectors, 0, 0));
  gsl_quat_float_set(pQ, 2, (float)gsl_matrix_get(eigenVectors, 1, 0));
  gsl_quat_float_set(pQ, 3, (float)gsl_matrix_get(eigenVectors, 2, 0));
  gsl_quat_float_set(pQ, 0, (float)gsl_matrix_get(eigenVectors, 3, 0));

  gsl_vector_free(eigenValues);
  gsl_matrix_free(eigenVectors);

  return 0;
}

int gsl_quat_float_toRotMatrix(gsl_quat_float *pQuat,
                               gsl_matrix_float *pRotMat) {
  if ((pRotMat->size1 != 3) || (pRotMat->size2 != 3)) {
    return -1;
  }

  gsl_matrix_float_set_zero(pRotMat);

  float q_w, q_x, q_y, q_z;
  q_w = gsl_quat_float_get(pQuat, 0);
  q_x = gsl_quat_float_get(pQuat, 1);
  q_y = gsl_quat_float_get(pQuat, 2);
  q_z = gsl_quat_float_get(pQuat, 3);

  gsl_matrix_float_set(pRotMat, 0, 0,
                       q_w * q_w + q_x * q_x - q_y * q_y - q_z * q_z);
  gsl_matrix_float_set(pRotMat, 0, 1, 2 * (q_x * q_y - q_z * q_w));
  gsl_matrix_float_set(pRotMat, 0, 2, 2 * (q_x * q_z + q_y * q_w));
  gsl_matrix_float_set(pRotMat, 1, 0, 2 * (q_x * q_y + q_z * q_w));
  gsl_matrix_float_set(pRotMat, 1, 1,
                       q_w * q_w - q_x * q_x + q_y * q_y - q_z * q_z);
  gsl_matrix_float_set(pRotMat, 1, 2, 2 * (q_y * q_z - q_x * q_w));
  gsl_matrix_float_set(pRotMat, 2, 0, 2 * (q_x * q_z - q_y * q_w));
  gsl_matrix_float_set(pRotMat, 2, 1, 2 * (q_y * q_z + q_x * q_w));
  gsl_matrix_float_set(pRotMat, 2, 2,
                       q_w * q_w + q_z * q_z - q_x * q_x - q_y * q_y);

  return 0;
}

void gsl_quat_float_toMatrix(gsl_quat_float *pQuat,
                             gsl_matrix_float *pQuatMat) {
  gsl_matrix_float_set_zero(pQuatMat);
  gsl_matrix_float *pMatrixT =
      gsl_matrix_float_calloc(pQuatMat->size1, pQuatMat->size2);

  gsl_matrix_float_set(pQuatMat, 0, 1, -gsl_quat_float_get(pQuat, 1));
  gsl_matrix_float_set(pQuatMat, 0, 2, -gsl_quat_float_get(pQuat, 2));
  gsl_matrix_float_set(pQuatMat, 0, 3, -gsl_quat_float_get(pQuat, 3));

  gsl_matrix_float_set(pQuatMat, 1, 2, gsl_quat_float_get(pQuat, 3));
  gsl_matrix_float_set(pQuatMat, 1, 3, -gsl_quat_float_get(pQuat, 2));

  gsl_matrix_float_set(pQuatMat, 2, 3, gsl_quat_float_get(pQuat, 1));

  gsl_matrix_float_transpose_memcpy(pMatrixT, pQuatMat);
  gsl_matrix_float_scale(pMatrixT, -1.f);

  gsl_matrix_float_add(pQuatMat, pMatrixT);
  gsl_matrix_float_free(pMatrixT);

  gsl_matrix_float_set(pQuatMat, 0, 0, gsl_quat_float_get(pQuat, 0));
  gsl_matrix_float_set(pQuatMat, 1, 1, gsl_quat_float_get(pQuat, 0));
  gsl_matrix_float_set(pQuatMat, 2, 2, gsl_quat_float_get(pQuat, 0));
  gsl_matrix_float_set(pQuatMat, 3, 3, gsl_quat_float_get(pQuat, 0));
}

void gsl_quat_float_rotvec(const gsl_quat_float *q, const gsl_vector_float *vIn,
                           gsl_vector_float *vRot) {
  gsl_quat_float *qV =
      gsl_quat_float_calloc();
  gsl_quat_float *qCopy =
      gsl_quat_float_calloc();
  gsl_quat_float_copy(q, qCopy);
  gsl_quat_float_fromVector(vIn, qV);

  gsl_quat_float_product(qV, qCopy);
  gsl_quat_float_conjugate(qCopy);
  gsl_quat_float_product(qCopy, qV);
  gsl_quat_float_get_imaginary(qCopy, vRot);

  gsl_quat_float_free(qV);
  gsl_quat_float_free(qCopy);
  
}

/**
 * Private functions definitions
 */
float fastSqrtf(float squaredNorm) {
  return (1 + squaredNorm) / (1 + 0.5 * squaredNorm);
}

gsl_matrix *rotMatGetK(gsl_matrix_float *pRotMat) {
  gsl_matrix *K = gsl_matrix_calloc(4, 4);
  gsl_matrix *KT = gsl_matrix_calloc(4, 4);
  double Q0, Q1, Q2;

  Q0 = gsl_matrix_float_get(pRotMat, 1, 0);
  Q1 = gsl_matrix_float_get(pRotMat, 0, 1);
  gsl_matrix_set(K, 0, 1, Q0 + Q1);

  Q0 = gsl_matrix_float_get(pRotMat, 2, 0);
  Q1 = gsl_matrix_float_get(pRotMat, 0, 2);
  gsl_matrix_set(K, 0, 2, Q0 + Q1);

  Q0 = gsl_matrix_float_get(pRotMat, 2, 1);
  Q1 = gsl_matrix_float_get(pRotMat, 1, 2);
  gsl_matrix_set(K, 0, 3, Q0 - Q1);

  Q0 = gsl_matrix_float_get(pRotMat, 2, 1);
  Q1 = gsl_matrix_float_get(pRotMat, 1, 2);
  gsl_matrix_set(K, 1, 2, Q0 + Q1);

  Q0 = gsl_matrix_float_get(pRotMat, 0, 2);
  Q1 = gsl_matrix_float_get(pRotMat, 2, 0);
  gsl_matrix_set(K, 1, 3, Q0 - Q1);

  Q0 = gsl_matrix_float_get(pRotMat, 1, 0);
  Q1 = gsl_matrix_float_get(pRotMat, 0, 1);
  gsl_matrix_set(K, 2, 3, Q0 - Q1);

  gsl_matrix_transpose_memcpy(KT, K);
  gsl_matrix_add(K, KT);

  gsl_matrix_free(KT);

  Q0 = gsl_matrix_float_get(pRotMat, 0, 0);
  Q1 = gsl_matrix_float_get(pRotMat, 1, 1);
  Q2 = gsl_matrix_float_get(pRotMat, 2, 2);
  gsl_matrix_set(K, 0, 0, Q0 - Q1 - Q2);
  gsl_matrix_set(K, 1, 1, Q1 - Q0 - Q2);
  gsl_matrix_set(K, 2, 2, Q2 - Q0 - Q2);
  gsl_matrix_set(K, 3, 3, Q0 + Q1 + Q2);

  gsl_matrix_scale(K, 1.f / 3.f);

  return K;
}