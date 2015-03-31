/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow/opticflow_calculator.c
 * @brief Estimate velocity from optic flow.
 *
 * Using images from a vertical camera and IMU sensor data.
 */

#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Own Header
#include "opticflow_calculator.h"

// Computer Vision
#include "vision/image.h"
#include "vision/lucas_kanade.h"
#include "vision/fastRosten.h"

// include polyfit
#include "math/pprz_polyfit_float.h"
#include "math/pprz_correlation.h"

// ARDrone Vertical Camera Parameters
#define FOV_H 0.67020643276
#define FOV_W 0.89360857702
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053

// Video Downlink options
//#define DOWNLINK_VIDEO 1         //to stream or not to stream
#define OPTICFLOW_SHOW_CORNERS 1 //to corner or not to corner
#define OPTICFLOW_SHOW_FLOW 1    //to flow or not to flow

// Check if settings are defined
#ifndef MAX_TRACK_CORNERS
#define MAX_TRACK_CORNERS 25
#endif
#ifndef HALF_WINDOW_SIZE
#define HALF_WINDOW_SIZE 5
#endif
#ifndef SUBPIXEL_FACTOR
#define SUBPIXEL_FACTOR 10
#endif
#ifndef MAX_ITERATIONS
#define MAX_ITERATIONS 10
#endif
#ifndef THRESHOLD_VEC
#define THRESHOLD_VEC 2
#endif

/* Variables only used here */
float x[MAX_TRACK_CORNERS];
float dx[MAX_TRACK_CORNERS];
float xdx_corr;

/* Functions only used here */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);
static int cmp_x(const void *a, const void *b);

/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 * @param[in] *w The image width
 * @param[in] *h The image height
 */
void opticflow_calc_init(struct opticflow_t *opticflow, uint16_t w, uint16_t h)
{
  /* Create the image buffers */
  image_create(&opticflow->img_gray, w, h, IMAGE_GRAYSCALE);
  image_create(&opticflow->prev_img_gray, w, h, IMAGE_GRAYSCALE);

  /* Set the previous values */
  opticflow->got_first_img = FALSE;
  opticflow->prev_psi = 0.0;
}

/**
 * Run the optical flow on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
void opticflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img, struct opticflow_result_t *result)
{
  // Update FPS for information
  result->fps = 1 / (timeval_diff(&opticflow->prev_timestamp, &img->ts) / 1000.);
  memcpy(&opticflow->prev_timestamp, &img->ts, sizeof(struct timeval));

  // Convert image to grayscale
  image_to_grayscale(img, &opticflow->img_gray);

  // Copy to previous image if not set
  if (!opticflow->got_first_img) {
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    opticflow->got_first_img = TRUE;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection (TODO: non fixed threashold)
  static uint8_t threshold = 20;
  struct point_t *corners = fast9_detect(img, threshold, 10, 20, 20, &result->corner_cnt);

  // Adaptive threshold
  if(result->corner_cnt < 40 && threshold > 5)
    threshold--;
  else if(result->corner_cnt > 50 && threshold < 60)
    threshold++;

#if DOWNLINK_VIDEO && OPTICFLOW_SHOW_CORNERS
  image_show_points(img, corners, result->corner_cnt);
#endif

  // Check if we found some corners to track
  if(result->corner_cnt < 1) {
    free(corners);
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    return;
  }

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************
  // Execute a Lucas Kanade optical flow
  result->tracked_cnt = result->corner_cnt;
  struct flow_t *vectors = opticFlowLK(&opticflow->img_gray, &opticflow->prev_img_gray, corners, &result->tracked_cnt,
    HALF_WINDOW_SIZE, SUBPIXEL_FACTOR, MAX_ITERATIONS, THRESHOLD_VEC, MAX_TRACK_CORNERS);

#if DOWNLINK_VIDEO && OPTICFLOW_SHOW_FLOW
  image_show_flow(img, vectors, result->tracked_cnt, SUBPIXEL_FACTOR);
#endif

  ///////////////////////////////////////////////////
  // Own Code //
  // State computatations (q and r from theta and psi)
  float diff_flow_x = (state->psi - opticflow->prev_psi) * img->w / FOV_W;
  opticflow->prev_psi = state->psi;

  // Sort the opticflow result based on x-location (Note: the array of structures is sorted, the content of each index is fixed)
  qsort(vectors, result->tracked_cnt, sizeof(struct flow_t), cmp_x);
  //printf("The diff_flow_x = %f and psi = %f\n", diff_flow_x, state->psi);
  for (int cnt = 0; cnt < result->tracked_cnt; cnt++){
    // Get x-location and flow from the results
    x[cnt] = vectors[cnt].pos.x/1.0;   //Get the x location of points and make into float
    dx[cnt] = vectors[cnt].flow_x/1.0; //Get the dx of the flow and make into float
  }

  // Get correlation between x and dx vectors
  result->xdx_corr = pprz_correlation(x, dx, result->tracked_cnt);
  //printf("The correlation of x and dx = %f\n", result->xdx_corr);

  for (int cnt = 0; cnt < result->tracked_cnt; cnt++){
    // Flow Derotation
    if (diff_flow_x < 0){
      dx[cnt] = dx[cnt] / (1-diff_flow_x);
    } else {
      dx[cnt] = dx[cnt] / (1+diff_flow_x);
    }
  }    

  // Polyfit the x and dx with a linear fit (and extract a_0 and a_1 coefficients)
  pprz_polyfit_float(x, dx, result->tracked_cnt, 1, result->points);
  ///////////////////////////////////////////////////

  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************
  free(corners);
  free(vectors);
  image_switch(&opticflow->img_gray, &opticflow->prev_img_gray);
}

/**
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 * @return The difference in milliseconds
 */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec=(finishtime->tv_sec-starttime->tv_sec)*1000;
  msec+=(finishtime->tv_usec-starttime->tv_usec)/1000;
  return msec;
}

/**
 * Compare two flow vectors based on x location
 * Used for sorting.
 * @param[in] *a The first flow vector (should be vect flow_t)
 * @param[in] *b The second flow vector (should be vect flow_t)
 * @return Negative if b has higher x location than a, 0 if the same and positive if a has higher x location than b
 */
static int cmp_x(const void *a, const void *b)
{
  const struct flow_t *a_p = (const struct flow_t *)a;
  const struct flow_t *b_p = (const struct flow_t *)b;
  return (a_p->pos.x) - (b_p->pos.x);
}
