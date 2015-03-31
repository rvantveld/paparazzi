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
 * Compute the correlation of two flow vectors (for all tracked corners)
 * @param[in] *x    The first vector (x)
 * @param[in] *y    The second vector (dx)
 * @param[in]  n    The number of tracked corners
 * @param[out] corr The correlation coefficient
 */

/* Include own header */
#include "pprz_correlation.h"

#include <math.h>

float pprz_correlation(float *x, float *y, unsigned int n){
        float xy[n], xsquare[n], ysquare[n];
        float xsum, ysum, xysum, xsqr_sum, ysqr_sum;
        float corr, num, den;

        xsum = ysum = xysum = xsqr_sum = ysqr_sum = 0;

        /* find the needed data to manipulate correlation coeff */
        for (int i = 0; i < n; i++) {
                xy[i] = x[i] * y[i];
                xsquare[i] = x[i] * x[i];
                ysquare[i] = y[i] * y[i];
                xsum = xsum + x[i];
                ysum = ysum + y[i];
                xysum = xysum + xy[i];
                xsqr_sum = xsqr_sum + xsquare[i];
                ysqr_sum = ysqr_sum + ysquare[i];
        }

        num = 1.0 * ((n * xysum) - (xsum * ysum));
        den = 1.0 * ((n * xsqr_sum - xsum * xsum)* (n * ysqr_sum - ysum * ysum));

        /* calculate correlation coefficient */
        corr = num / sqrt(den);

        return corr;
}
