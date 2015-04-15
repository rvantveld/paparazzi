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
 *
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[in] *downsize_factor The downsample factor used to reduce computational effort
 * @param[in] *y_m Threshold regarding minimum luminance
 * @param[in] *y_M Threshold regarding maximum luminance
 * @param[in] *u_m Threshold regarding minimum blue difference chroma
 * @param[in] *u_M Threshold regarding maximum blue difference chroma
 * @param[in] *v_m Threshold regarding minimum red difference chroma
 * @param[in] *v_M Threshold regarding maximum red difference chroma
 * @param[out] *color_counted The amount of pixels of specified color in downsampled image
 */

// Own header
#include "color_count.h"

// Image processing
#include "vision/image.h"

uint16_t color_count(struct image_t *img, uint8_t downsize_factor, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M)
{
  // Initialize smaller image for resizing
  struct image_t small;
  image_create(&small, img->w/downsize_factor, img->h/downsize_factor, IMAGE_YUV422);

  // Resize: device with downsize_factor using downsampling
  image_yuv422_downsample(img, &small, downsize_factor);
  
  // Color count function, using downsampled image  
  color_counted = image_yuv422_colorfilt(&small, &small, 
      y_m, y_M, u_m, u_M, v_m, v_M);

  // Free the smaller image
  image_free(&small);
  return color_counted;
}