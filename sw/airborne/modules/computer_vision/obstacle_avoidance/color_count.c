/*
 * Copyright (C) 2015
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
 * @file color_count.c
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
    
  color_counted = image_yuv422_colorfilt(&small, &small, 
      y_m, y_M, u_m, u_M, v_m, v_M);
    
  printf("ColorCount = %d \n", color_counted);

  // Free the smaller image
  image_free(&small);
  return color_counted;
}
