/* statistics/gsl_statistics_ulong.h
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

#ifndef __GSL_STATISTICS_ULONG_H__
#define __GSL_STATISTICS_ULONG_H__

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

double gsl_stats_ulong_mean (const unsigned long data[], const size_t stride, const size_t n);
double gsl_stats_ulong_variance (const unsigned long data[], const size_t stride, const size_t n);
double gsl_stats_ulong_sd (const unsigned long data[], const size_t stride, const size_t n);
double gsl_stats_ulong_variance_with_fixed_mean (const unsigned long data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_ulong_sd_with_fixed_mean (const unsigned long data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_ulong_tss (const unsigned long data[], const size_t stride, const size_t n);
double gsl_stats_ulong_tss_m (const unsigned long data[], const size_t stride, const size_t n, const double mean);

double gsl_stats_ulong_absdev (const unsigned long data[], const size_t stride, const size_t n);
double gsl_stats_ulong_skew (const unsigned long data[], const size_t stride, const size_t n);
double gsl_stats_ulong_kurtosis (const unsigned long data[], const size_t stride, const size_t n);
double gsl_stats_ulong_lag1_autocorrelation (const unsigned long data[], const size_t stride, const size_t n);

double gsl_stats_ulong_covariance (const unsigned long data1[], const size_t stride1,const unsigned long data2[], const size_t stride2, const size_t n);
double gsl_stats_ulong_correlation (const unsigned long data1[], const size_t stride1,const unsigned long data2[], const size_t stride2, const size_t n);
double gsl_stats_ulong_spearman (const unsigned long data1[], const size_t stride1, const unsigned long data2[], const size_t stride2, const size_t n, double work[]);

double gsl_stats_ulong_variance_m (const unsigned long data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_ulong_sd_m (const unsigned long data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_ulong_absdev_m (const unsigned long data[], const size_t stride, const size_t n, const double mean);
double gsl_stats_ulong_skew_m_sd (const unsigned long data[], const size_t stride, const size_t n, const double mean, const double sd);
double gsl_stats_ulong_kurtosis_m_sd (const unsigned long data[], const size_t stride, const size_t n, const double mean, const double sd);
double gsl_stats_ulong_lag1_autocorrelation_m (const unsigned long data[], const size_t stride, const size_t n, const double mean);

double gsl_stats_ulong_covariance_m (const unsigned long data1[], const size_t stride1,const unsigned long data2[], const size_t stride2, const size_t n, const double mean1, const double mean2);


double gsl_stats_ulong_pvariance (const unsigned long data1[], const size_t stride1, const size_t n1, const unsigned long data2[], const size_t stride2, const size_t n2);
double gsl_stats_ulong_ttest (const unsigned long data1[], const size_t stride1, const size_t n1, const unsigned long data2[], const size_t stride2, const size_t n2);

unsigned long gsl_stats_ulong_max (const unsigned long data[], const size_t stride, const size_t n);
unsigned long gsl_stats_ulong_min (const unsigned long data[], const size_t stride, const size_t n);
void gsl_stats_ulong_minmax (unsigned long * min, unsigned long * max, const unsigned long data[], const size_t stride, const size_t n);

size_t gsl_stats_ulong_max_index (const unsigned long data[], const size_t stride, const size_t n);
size_t gsl_stats_ulong_min_index (const unsigned long data[], const size_t stride, const size_t n);
void gsl_stats_ulong_minmax_index (size_t * min_index, size_t * max_index, const unsigned long data[], const size_t stride, const size_t n);

unsigned long gsl_stats_ulong_select(unsigned long data[], const size_t stride, const size_t n, const size_t k);

double gsl_stats_ulong_median_from_sorted_data (const unsigned long sorted_data[], const size_t stride, const size_t n) ;
double gsl_stats_ulong_median (unsigned long sorted_data[], const size_t stride, const size_t n);
double gsl_stats_ulong_quantile_from_sorted_data (const unsigned long sorted_data[], const size_t stride, const size_t n, const double f) ;

double gsl_stats_ulong_trmean_from_sorted_data (const double trim, const unsigned long sorted_data[], const size_t stride, const size_t n) ;
double gsl_stats_ulong_gastwirth_from_sorted_data (const unsigned long sorted_data[], const size_t stride, const size_t n) ;

double gsl_stats_ulong_mad0(const unsigned long data[], const size_t stride, const size_t n, double work[]);
double gsl_stats_ulong_mad(const unsigned long data[], const size_t stride, const size_t n, double work[]);

unsigned long gsl_stats_ulong_Sn0_from_sorted_data (const unsigned long sorted_data[], const size_t stride, const size_t n, unsigned long work[]) ;
double gsl_stats_ulong_Sn_from_sorted_data (const unsigned long sorted_data[], const size_t stride, const size_t n, unsigned long work[]) ;

unsigned long gsl_stats_ulong_Qn0_from_sorted_data (const unsigned long sorted_data[], const size_t stride, const size_t n, unsigned long work[], int work_int[]) ;
double gsl_stats_ulong_Qn_from_sorted_data (const unsigned long sorted_data[], const size_t stride, const size_t n, unsigned long work[], int work_int[]) ;

__END_DECLS

#endif /* __GSL_STATISTICS_ULONG_H__ */
