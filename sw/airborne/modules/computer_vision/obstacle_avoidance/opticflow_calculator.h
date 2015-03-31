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
 * @file modules/computer_vision/opticflow/opticflow_calculator.h
 * @brief Calculate velocity from optic flow.
 *
 * Using images from a vertical camera and IMU sensor data.
 */

#ifndef OPTICFLOW_CALCULATOR_H
#define OPTICFLOW_CALCULATOR_H

#include "std.h"
#include "inter_thread_data.h"
#include "vision/image.h"
#include "v4l/v4l2.h"

struct opticflow_t
{
  bool_t got_first_img;             //< If we got a image to work with
  float prev_phi;                   //< Phi from the previous image frame
  float prev_theta;                 //< Theta from the previous image frame
  float prev_psi;                   //< Psi from the previous image frame
  struct image_t small;             //< Current small (downsampled) image frame
  struct image_t img_gray;          //< Current gray image frame
  struct image_t prev_img_gray;     //< Previous gray image frame
  struct timeval prev_timestamp;    //< Timestamp of the previous frame, used for FPS calculation
};


void opticflow_calc_init(struct opticflow_t *opticflow, uint16_t w, uint16_t h);
void opticflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *state, struct image_t *img, struct opticflow_result_t *result);
float correlation(float *x, float *y, uint16_t n);

#endif /* OPTICFLOW_CALCULATOR_H */
