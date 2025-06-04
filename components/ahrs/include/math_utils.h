#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include "gsl/gsl_matrix_float.h"
#include "gsl/gsl_vector_float.h"
#include <inttypes.h>

/**
 * @brief Creates a skew symetric matrix from a vector
 *
 * @param pV Pointer of the source vector
 * @param pM Pointer to the result matrix
 */
extern int skewSymFromVector(gsl_vector_float *pV, gsl_matrix_float *pM);

extern int8_t cross_product(const gsl_vector_float *a, const gsl_vector_float *b,
                          gsl_vector_float *c);

extern int8_t gsl_matrix_float2double(const gsl_matrix_float *m_float,
                                    gsl_matrix *m_double);
extern int8_t gsl_matrix_double2float(const gsl_matrix *m_double,
                                    gsl_matrix_float *m_float);
extern int8_t gsl_vector_float2double(const gsl_vector_float *v_float,
                                    gsl_vector *v_double);
extern int8_t gsl_vector_double2float(const gsl_vector *v_double,
                                    gsl_vector_float *v_float);

extern int8_t gsl_matrix_issymetric(const gsl_matrix *m);          
extern int8_t gsl_matrix_force_symmetric(gsl_matrix *m);     

extern int8_t gsl_matrix_float_issymetric(const gsl_matrix *m);          
extern int8_t gsl_matrix_float_force_symmetric(gsl_matrix *m); 

/* *****************************************************************************
A demonstration for calculating pseudo-inverse matrix using the GNU Scientific
Library (GSL).
Copyright (C) 2025 by Akira TAMAMORI
This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA.
***************************************************************************** */

/* *****************************************************************************
This program utilizes the GNU Scientific Library (GSL).  GSL is licensed under
the GNU General Public License version 3 or later.  See the GSL documentation or
source code for its copyright notice.
***************************************************************************** */
extern int8_t gsl_double_pinv(const gsl_matrix *M, double tolerance_level,
                       gsl_matrix *M_pinv);

extern int8_t normal_dist_intersection(const gsl_vector_float *v1,
                           const gsl_vector_float *v2, gsl_vector_float *v3,
                           const gsl_matrix_float *P1,
                           const gsl_matrix_float *P2, gsl_matrix_float *P3);
#endif