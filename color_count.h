/*
 * Copyright (C) 2015 Group 1 AE4317 TU Delft
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/obstacle_avoidance/color_count.h
 * @brief Count amount of pixel of specified color
 *
 * Count amount of pixels of a specified color for the given image
 * The image can be downsampled to reduce the computational effort
 * The color is specified with a minimum and maximum threshold
 * for the Y,U and V channels of the UYUV image format
 */

#ifndef COLOR_COUNT_H
#define COLOR_COUNT_H

#include "vision/image.h"
#include <stdio.h>

// Module functions
uint16_t color_count(struct image_t *img, uint8_t downsize_factor, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M);

// Module variables
extern uint16_t color_counted;

#endif /* COLOR_COUNT_H */
