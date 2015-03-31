/*
 * Copyright (C) 2014 Ronald van 't Veld
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
 */

#ifndef PPRZ_CORRELATION_H
#define PPRZ_CORRELATION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Compute the correlation of two flow vectors (for all tracked corners)
 * @param[in] *x    The first vector (x)
 * @param[in] *y    The second vector (dx)
 * @param[in]  n    The number of tracked corners
 * @param[out] corr The correlation coefficient
 */

float pprz_correlation(float *x, float *y, unsigned int n);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_CORRELATION_H */
