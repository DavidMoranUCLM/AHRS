#include "math_utils.h"

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <inttypes.h>

#include "gsl/gsl_matrix_float.h"
#include "gsl/gsl_vector_float.h"

int skewSymFromVector(gsl_vector_float *pV, gsl_matrix_float *pM) {
  if ((pV->size != pM->size1) || (pV->size != pM->size2)) {
    return -1;
  }

  if (pV->size != 3) {
    return -1;
  }

  gsl_matrix_float_set_zero(pM);

  gsl_matrix_float_set(pM, 0, 1, -gsl_vector_float_get(pV, 2));
  gsl_matrix_float_set(pM, 0, 2, gsl_vector_float_get(pV, 1));
  gsl_matrix_float_set(pM, 1, 2, -gsl_vector_float_get(pV, 0));

  gsl_matrix_float *pMT = gsl_matrix_float_alloc(3, 3);
  gsl_matrix_float_transpose_memcpy(pMT, pM);
  gsl_matrix_float_scale(pMT, -1.f);

  gsl_matrix_float_add(pM, pMT);
  gsl_matrix_float_free(pMT);

  return 0;
}

int8_t cross_product(const gsl_vector_float *a, const gsl_vector_float *b,
                     gsl_vector_float *c) {
  if (a->size != 3 || b->size != 3 || c->size != 3) {
    fprintf(stderr, "Error: Vectors must have size 3.\n");
    return -1;
  }

  float c1 = gsl_vector_float_get(a, 1) * gsl_vector_float_get(b, 2) -
             gsl_vector_float_get(a, 2) * gsl_vector_float_get(b, 1);
  float c2 = gsl_vector_float_get(a, 2) * gsl_vector_float_get(b, 0) -
             gsl_vector_float_get(a, 0) * gsl_vector_float_get(b, 2);
  float c3 = gsl_vector_float_get(a, 0) * gsl_vector_float_get(b, 1) -
             gsl_vector_float_get(a, 1) * gsl_vector_float_get(b, 0);

  gsl_vector_float_set(c, 0, c1);
  gsl_vector_float_set(c, 1, c2);
  gsl_vector_float_set(c, 2, c3);

  return 0;
}

int8_t gsl_matrix_float2double(const gsl_matrix_float *m_float,
                               gsl_matrix *m_double) {
  if (m_float->size1 != m_double->size1 || m_float->size2 != m_double->size2) {
    return GSL_EBADLEN;
  }
  for (size_t i = 0; i < m_float->size1; ++i) {
    for (size_t j = 0; j < m_float->size2; ++j) {
      gsl_matrix_set(m_double, i, j,
                     (double)gsl_matrix_float_get(m_float, i, j));
    }
  }
  return 0;
}

int8_t gsl_matrix_double2float(const gsl_matrix *m_double,
                               gsl_matrix_float *m_float) {
  if (m_double->size1 != m_float->size1 || m_double->size2 != m_float->size2) {
    return GSL_EBADLEN;
  }
  for (size_t i = 0; i < m_double->size1; ++i) {
    for (size_t j = 0; j < m_double->size2; ++j) {
      gsl_matrix_float_set(m_float, i, j,
                           (float)gsl_matrix_get(m_double, i, j));
    }
  }
  return 0;
}

int8_t gsl_vector_float2double(const gsl_vector_float *v_float,
                               gsl_vector *v_double) {
  if (v_float->size != v_double->size) {
    return GSL_EBADLEN;
  }
  for (size_t i = 0; i < v_float->size; ++i) {
    gsl_vector_set(v_double, i, (double)gsl_vector_float_get(v_float, i));
  }
  return 0;
}

int8_t gsl_vector_double2float(const gsl_vector *v_double,
                               gsl_vector_float *v_float) {
  if (v_double->size != v_float->size) {
    return GSL_EBADLEN;
  }
  for (size_t i = 0; i < v_double->size; ++i) {
    gsl_vector_float_set(v_float, i, (float)gsl_vector_get(v_double, i));
  }
  return 0;
}

int8_t gsl_matrix_issymetric(const gsl_matrix *m) {
  if (m->size1 != m->size2) {
    return GSL_FAILURE;
  }
  for (size_t i = 0; i < m->size1; ++i) {
    for (size_t j = i + 1; j < m->size2; ++j) {
      if (abs(gsl_matrix_get(m, i, j) - gsl_matrix_get(m, j, i)) >
          GSL_DBL_EPSILON) {
        return GSL_FAILURE;
      }
    }
  }
  return GSL_SUCCESS;
}

int8_t gsl_matrix_force_symmetric(gsl_matrix *m) {
  if (m->size1 != m->size2) {
    return GSL_FAILURE;
  }
  if (gsl_matrix_issymetric(m) == GSL_SUCCESS) return GSL_SUCCESS;
  float elem = 0;
  for (uint8_t i = 0; i < m->size1; i++) {
    for (uint8_t j = 0; i < m->size2; j++) {
      if (i > j) {
        elem += gsl_matrix_get(m, i, j);
        elem += gsl_matrix_get(m, j, i);
        elem *= 0.5f;
        gsl_matrix_set(m, i, j, elem);
        gsl_matrix_set(m, j, i, elem);
        elem = 0;
      }
    }
  }
  return GSL_SUCCESS;
}
int8_t gsl_matrix_float_issymetric(const gsl_matrix *m) {
  if (m->size1 != m->size2) {
    return GSL_FAILURE;
  }
  for (size_t i = 0; i < m->size1; ++i) {
    for (size_t j = i + 1; j < m->size2; ++j) {
      if (abs(gsl_matrix_float_get(m, i, j) - gsl_matrix_float_get(m, j, i)) >
          GSL_DBL_EPSILON) {
        return GSL_FAILURE;
      }
    }
  }
  return GSL_SUCCESS;
}
int8_t gsl_matrix_float_force_symmetric(gsl_matrix *m) {
  if (m->size1 != m->size2) {
    return GSL_FAILURE;
  }
  if (gsl_matrix_float_issymetric(m) == GSL_SUCCESS) return GSL_SUCCESS;
  float elem = 0;
  for (uint8_t i = 0; i < m->size1; i++) {
    for (uint8_t j = 0; i < m->size2; j++) {
      if (i > j) {
        elem += gsl_matrix_float_get(m, i, j);
        elem += gsl_matrix_float_get(m, j, i);
        elem *= 0.5f;
        gsl_matrix_float_set(m, i, j, elem);
        gsl_matrix_float_set(m, j, i, elem);
        elem = 0;
      }
    }
  }
  return GSL_SUCCESS;
}

int8_t gsl_double_pinv(const gsl_matrix *M, double tolerance_level,
                       gsl_matrix *M_pinv) {
  /* Calculate the pseudo-inverse for a given matrix.
  Args:
  M (const gsl_matrix *): The original matrix.
  tolerance_level (double): The tolerance level of singular value.
  M_pinv (gsl_matrix *): The pseudo-inverse of M.
  */

  int m = M->size1;
  int n = M->size2;
  gsl_matrix *U = gsl_matrix_alloc(m, n);
  gsl_matrix *V = gsl_matrix_alloc(n, n);
  gsl_vector *S = gsl_vector_alloc(n);
  gsl_vector *work = gsl_vector_alloc(n);
  gsl_vector *col_vector = gsl_vector_alloc(m);
  gsl_matrix_memcpy(U, M);
  gsl_linalg_SV_decomp(U, V, S, work);
  tolerance_level = GSL_MAX(m, n) * GSL_DBL_EPSILON * gsl_vector_max(S);
  for (int i = 0; i < n; i++) {
    double s = gsl_vector_get(S, i);
    gsl_matrix_get_col(col_vector, U, i);
    if (s > tolerance_level) {
      gsl_vector_scale(col_vector, 1.0 / s);
    } else {
      gsl_vector_set_zero(col_vector);
    }
    gsl_matrix_set_col(U, i, col_vector);
  }
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, V, U, 0.0, M_pinv);

  int is_sym = gsl_matrix_issymetric(M);
  if (is_sym == GSL_SUCCESS) {
    gsl_matrix *M_pinv_T = gsl_matrix_alloc(m, n);
    gsl_matrix_transpose_memcpy(M_pinv_T, M_pinv);
    gsl_matrix_add(M_pinv, M_pinv_T);
    gsl_matrix_scale(M_pinv, 0.5f);
    gsl_matrix_free(M_pinv_T);
  }

  gsl_matrix_free(U);
  gsl_matrix_free(V);
  gsl_vector_free(S);
  gsl_vector_free(work);
  gsl_vector_free(col_vector);

  return 0;
}

int8_t normal_dist_intersection(const gsl_vector_float *v1,
                                const gsl_vector_float *v2,
                                gsl_vector_float *v3,
                                const gsl_matrix_float *P1,
                                const gsl_matrix_float *P2,
                                gsl_matrix_float *P3) {
  uint8_t M = P1->size1;
  gsl_matrix *P1_d = gsl_matrix_alloc(M, M);
  gsl_matrix *P2_d = gsl_matrix_alloc(M, M);

  gsl_matrix *P1_inv_d = gsl_matrix_alloc(M, M);
  gsl_matrix *P2_inv_d = gsl_matrix_alloc(M, M);
  gsl_matrix *P3_inv_d = gsl_matrix_calloc(M, M);
  gsl_matrix *H_d = gsl_matrix_calloc(M, M);

  gsl_vector *v1_d = gsl_vector_alloc(M);
  gsl_vector *v2_d = gsl_vector_alloc(M);
  gsl_vector *v3_d = gsl_vector_alloc(M);
  gsl_vector *y_d = gsl_vector_alloc(M);

  gsl_vector_float2double(v1, v1_d);
  gsl_vector_float2double(v2, v2_d);

  gsl_matrix_float2double(P1, P1_d);
  gsl_matrix_float2double(P2, P2_d);

  gsl_double_pinv(P1_d, 1e-32, P1_inv_d);
  gsl_double_pinv(P2_d, 1e-32, P2_inv_d);

  // y_d = P1_inv_d * v1_d + P2_inv_d * v2_d
  gsl_blas_dsymv(CblasUpper, 1.0, P1_inv_d, v1_d, 0.0, y_d);
  gsl_blas_dsymv(CblasUpper, 1.0, P2_inv_d, v2_d, 1, y_d);

  // H = P1_inv_d + P2_inv_d
  gsl_matrix_memcpy(H_d, P1_inv_d);
  gsl_matrix_add(H_d, P2_inv_d);

  // P3_d = P3_inv_d^-1
  gsl_matrix_memcpy(P1_inv_d, H_d);
  gsl_error_handler_t *err_hand = gsl_set_error_handler_off();
  int gsl_status = gsl_linalg_cholesky_decomp1(H_d);
  gsl_set_error_handler(err_hand);

  if (gsl_status == GSL_SUCCESS) {
    gsl_linalg_cholesky_invert(H_d);
  } else {
    gsl_double_pinv(P1_inv_d, 1e-32, H_d);
  }
  gsl_matrix_double2float(H_d, P3);

  // v3_d = P3_inv_d * y_d
  gsl_blas_dsymv(CblasUpper, 1.0, H_d, y_d, 0.0, v3_d);
  gsl_vector_double2float(v3_d, v3);

  gsl_matrix_free(P1_d);
  gsl_matrix_free(P2_d);
  gsl_matrix_free(P1_inv_d);
  gsl_matrix_free(P2_inv_d);
  gsl_matrix_free(P3_inv_d);
  gsl_matrix_free(H_d);
  gsl_vector_free(v1_d);
  gsl_vector_free(v2_d);
  gsl_vector_free(v3_d);
  gsl_vector_free(y_d);

  return 0;
}