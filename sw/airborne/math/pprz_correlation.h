/*
 * Copyright (C) 2015 Ronald van 't Veld
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file math/pprz_correlation.h
 * @brief Compute correlation.
 *
 * Function to compute the Pearson Product-Moment correlation coefficient of two vectors
 * http://en.wikipedia.org/wiki/Pearson_product-moment_correlation_coefficient
 * In statistics, the Pearson product-moment correlation coefficient is a measure of the     
 * linear correlation (dependence) between two variables X and Y, giving a value between 
 * +1 and −1 inclusive, where 1 is total positive correlation, 0 is no correlation, and 
 * −1 is total negative correlation. It is widely used in the sciences as a measure of 
 * the degree of linear dependence between two variables. It was developed by Karl Pearson 
 * from a related idea introduced by Francis Galton in the 1880s.
 *
 * Compute the correlation of two vectors
 * @param[in] *x    Pointer to the first vector (float)
 * @param[in] *y    Pointer to the second vector (float)
 * @param[in]  n    The length of both vectors (uint)
 * @param[out] corr The correlation coefficient
 */

#ifndef PPRZ_CORRELATION_H
#define PPRZ_CORRELATION_H

#ifdef __cplusplus
extern "C" {
#endif

float pprz_correlation(float *x, float *y, unsigned int n);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_CORRELATION_H */
