/**
 * Compute the correlation of two flow vectors (for all tracked corners)
 * @param[in] *x    The first vector (x)
 * @param[in] *y    The second vector (dx)
 * @param[in]  n    The number of tracked corners
 * @param[out] corr The correlation coefficient
 */
float correlation(float *x, float *y, uint16_t n){
        float xy[n], xsquare[n], ysquare[n];
        int xsum, ysum, xysum, xsqr_sum, ysqr_sum;
        float corr, num, deno;

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
        deno = 1.0 * ((n * xsqr_sum - xsum * xsum)* (n * ysqr_sum - ysum * ysum));

        /* calculate correlation coefficient */
        corr = num / sqrt(deno);

        return corr;
}
