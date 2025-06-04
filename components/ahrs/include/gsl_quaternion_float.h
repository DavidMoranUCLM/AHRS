#ifndef GSL_QUATERNION_FLOAT_H
#define GSL_QUATERNION_FLOAT_H

#include <inttypes.h>
#include <stdlib.h>

#include "gsl/gsl_matrix_float.h"
#include "gsl/gsl_vector_float.h"

typedef gsl_vector_float gsl_quat_float;

/**
 * @return Pointer to an allocated quat
 */
extern gsl_quat_float* gsl_quat_float_alloc(void);

/**
 * @return Pointer to an allocated quat initialized to 0
 */
extern gsl_quat_float* gsl_quat_float_calloc(void);

/**
 * @param[in] pQ Pointer to the quat to free from heap
 */
extern void gsl_quat_float_free(gsl_quat_float* pQ);

/**
 * @param[in] pQ Pointer to the quat to set value
 * @param[in] i Elem to set
 * @param[in] value Value to set
 */
extern void gsl_quat_float_set(gsl_quat_float* pQ, uint8_t i, float value);

/**
 * @param[in] pQ Pointer to the quat to get value
 * @param[in] i Elem to set
 * @return Float i element
 */
extern float gsl_quat_float_get(gsl_quat_float* pQ, uint8_t i);

/**
 * @param[in] pQ Pointer to the quat to get imaginary part
 * @param[out] pV Pointer to the vector to store imaginary part
 */
extern int gsl_quat_float_get_imaginary(gsl_quat_float* pQ, gsl_vector_float *pV);

/**
 * @brief Gets the norm of pQ
 * @param pQ Pointer to the quat to calculate the norm of
 * @return Float type with the norm of the quat
 */
extern float gsl_quat_float_norm(gsl_quat_float* pQ);

/**
 * @brief Normalizes pQ
 * @param pQ Pointer to the quat to normalize
 */
extern void gsl_quat_float_normalize(gsl_quat_float* pQ);

/**
 * @brief Gets the conjugate of pQ
 * @param pQ[in] Pointer to the quat to calculate the conjugate of
 * @param pQ[out] Conjugate
 */
extern void gsl_quat_float_conjugate(gsl_quat_float* pQ);

/**
 * @brief Copies the data of pSrc to pDst
 * @param pSrc Pointer to the quat to copy from
 * @param pDst Pointer to the quat to copy to
 */
extern void gsl_quat_float_copy(const gsl_quat_float* pSrc, gsl_quat_float* pDst);

/**
 * @brief Performs the product pQ1 * pQ2 and gives result by pQ1
 * @param pQ1
 * @param pQ2
 */
extern void gsl_quat_float_product(gsl_quat_float* pQ1,
                                              const gsl_quat_float* pQ2);

/**
 * @brief Performs the inverse of pQ1, a unit quat
 * @param pQ[in] Pointer to the quat to calculate the inverse of
 * @param pQ[out] The inverse
 */
extern int gsl_quat_float_inv(gsl_quat_float* pQ);

/**
 * @brief Creates a quat from a Axis vector and a Angle in radians
 * @param pAxis Pointer to the vector to use as an axis
 * @param angleRad Angle in radians
 * @param pQ Pointer to the created quat
 */
extern int gsl_quat_float_fromAxis(gsl_vector_float* pAxis,
                                               const float angleRad, gsl_quat_float *pQ);

/**
 * @brief Creates a quat [0, v0, v1, v2] from a vector [v0, v1, v2]
 * @param pAxis Pointer to the vector to use as an axis
 * @param pQ Pointer to the created quat
 */
extern int gsl_quat_float_fromVector(gsl_vector_float* pVector, gsl_quat_float *pQ);


/**
 * @brief Creates a quaternion from Euler angles (roll, pitch, yaw)
 * 
 * @param roll Roll angle in radians
 * @param pitch Pitch angle in radians
 * @param yaw Yaw angle in radians
 * @param pQ Pointer to the created quaternion
 * @return int 0 if successful, non-zero otherwise
 */
extern int gsl_quat_float_fromEuler(float roll, float pitch, float yaw, gsl_quat_float *pQ);

/**
 * @brief Gets the closest quat equivalent to the rotation of pRotMat
 * @param pQ pointer to quat
 * @param pRotMat pinter to 3x3 rotation matrix
 */
extern int gsl_quat_float_fromRotMatrix(
    gsl_matrix_float* pRotMat, gsl_quat_float *pQ);

/**
 * @brief Creates a 3x3 rotation matrix equivalent a quat
 * @param pQuat[in] pointer to quat
 * @param pQuatMat[out] Pointer to  matrix
 */
extern int gsl_quat_float_toRotMatrix(gsl_quat_float* pQuat, gsl_matrix_float *pRotMat);

/**
 * @brief Creates a 4x4 matrix equivalent to a linear operation with  a quat
 * @param pQuat[in] pointer to quat
 * @param pQuatMat[out] Pointer to matrix
 */
extern void gsl_quat_float_toMatrix(gsl_quat_float* pQuat, gsl_matrix_float *pQuatMat);

/**
 * @brief Rotates a vector by a quat
 * @param pQ Pointer to the quat to rotate with
 * @param pVIn Pointer to the vector to rotate
 * @param pVRot Pointer to the rotated vector
 */
extern void gsl_quat_float_rotvec(const gsl_quat_float* pQ,
                                   const gsl_vector_float* pVIn,
                                   gsl_vector_float* pVRot);

#endif