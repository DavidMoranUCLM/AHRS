/* statistics/gsl_statistics_uint.h
 * 
 * Copyright (C) 1996, 1997, 1998, 1999, 2000, 2007 Jim Davies, Brian Gough
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or (at
 * your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef __GSL_STATISTICS_UINT_H__
#define __GSL_STATISTICS_UINT_H__

#include <stddef.h>
#include <stdlib.h>

#undef __BEGIN_DECLS
#undef __END_DECLS
#ifdef __cplusplus
# define __BEGIN_DECLS extern "C" {
# define __END_DECLS }
#else
# define __BEGIN_DECLS /* empty */
# define __END_DECLS /* empty */
#endif

__BEGIN_DECLS

double gsl_stats_uint_mean (const unsigned int data[], const size_t stride, const size_t n);
double gsl_stats_uint_variance (const unsigned int data[], const size_t stride, const size_t n);
double gsl_stats_uint_sd (const unsigned int data[], const size_t stride, const size_t n);
double gsl_stats_uint_variance_with_fixed_mean (const unsigned int data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_uint_sd_with_fixed_mean (const unsigned int data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_uint_tss (const unsigned int data[], const size_t stride, const size_t n);
double gsl_stats_uint_tss_m (const unsigned int data[], const size_t stride, const size_t n, const double mean);

double gsl_stats_uint_absdev (const unsigned int data[], const size_t stride, const size_t n);
double gsl_stats_uint_skew (const unsigned int data[], const size_t stride, const size_t n);
double gsl_stats_uint_kurtosis (const unsigned int data[], const size_t stride, const size_t n);
double gsl_stats_uint_lag1_autocorrelation (const unsigned int data[], const size_t stride, const size_t n);

double gsl_stats_uint_covariance (const unsigned int data1[], const size_t stride1,const unsigned int data2[], const size_t stride2, const size_t n);
double gsl_stats_uint_correlation (const unsigned int data1[], const size_t stride1,const unsigned int data2[], const size_t stride2, const size_t n);
double gsl_stats_uint_spearman (const unsigned int data1[], const size_t stride1, const unsigned int data2[], const size_t stride2, const size_t n, double work[]);

double gsl_stats_uint_variance_m (const unsigned int data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_uint_sd_m (const unsigned int data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_uint_absdev_m (const unsigned int data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_uint_skew_m_sd (const unsigned int data[], const size_t stride, const size_t n, const double mean, const double sd);
double gsl_stats_uint_kurtosis_m_sd (const unsigned int data[], const size_t stride, const size_t n, const double mean, const double sd);
double gsl_stats_uint_lag1_autocorrelation_m (const unsigned int data[], const size_t stride, const size_t n, const double mean);

double gsl_stats_uint_covariance_m (const unsigned int data1[], const size_t stride1,const unsigned int data2[], const size_t stride2, const size_t n, const double mean1, const double mean2);


double gsl_stats_uint_pvariance (const unsigned int data1[], const size_t stride1, const size_t n1, const unsigned int data2[], const size_t stride2, const size_t n2);
double gsl_stats_uint_ttest (const unsigned int data1[], const size_t stride1, const size_t n1, const unsigned int data2[], const size_t stride2, const size_t n2);

unsigned int gsl_stats_uint_max (const unsigned int data[], const size_t stride, const size_t n);
unsigned int gsl_stats_uint_min (const unsigned int data[], const size_t stride, const size_t n);
void gsl_stats_uint_minmax (unsigned int * min, unsigned int * max, const unsigned int data[], const size_t stride, const size_t n);

size_t gsl_stats_uint_max_index (const unsigned int data[], const size_t stride, const size_t n);
size_t gsl_stats_uint_min_index (const unsigned int data[], const size_t stride, const size_t n);
void gsl_stats_uint_minmax_index (size_t * min_index, size_t * max_index, const unsigned int data[], const size_t stride, const size_t n);

unsigned int gsl_stats_uint_select(unsigned int data[], const size_t stride, const size_t n, const size_t k);

double gsl_stats_uint_median_from_sorted_data (const unsigned int sorted_data[], const size_t stride, const size_t n) ;
double gsl_stats_uint_median (unsigned int sorted_data[], const size_t stride, const size_t n);
double gsl_stats_uint_quantile_from_sorted_data (const unsigned int sorted_data[], const size_t stride, const size_t n, const double f) ;

double gsl_stats_uint_trmean_from_sorted_data (const double trim, const unsigned int sorted_data[], const size_t stride, const size_t n) ;
double gsl_stats_uint_gastwirth_from_sorted_data (const unsigned int sorted_data[], const size_t stride, const size_t n) ;

double gsl_stats_uint_mad0(const unsigned int data[], const size_t stride, const size_t n, double work[]);
double gsl_stats_uint_mad(const unsigned int data[], const size_t stride, const size_t n, double work[]);

unsigned int gsl_stats_uint_Sn0_from_sorted_data (const unsigned int sorted_data[], const size_t stride, const size_t n, unsigned int work[]) ;
double gsl_stats_uint_Sn_from_sorted_data (const unsigned int sorted_data[], const size_t stride, const size_t n, unsigned int work[]) ;

unsigned int gsl_stats_uint_Qn0_from_sorted_data (const unsigned int sorted_data[], const size_t stride, const size_t n, unsigned int work[], int work_int[]) ;
double gsl_stats_uint_Qn_from_sorted_data (const unsigned int sorted_data[], const size_t stride, const size_t n, unsigned int work[], int work_int[]) ;

__END_DECLS

#endif /* __GSL_STATISTICS_UINT_H__ */
