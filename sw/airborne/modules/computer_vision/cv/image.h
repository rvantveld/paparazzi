/*
 * Copyright (C) 2012-2013
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


#ifndef _MY_IMAGE_HEADER_
#define _MY_IMAGE_HEADER_

/* V4L2 memory mapped image buffer */
struct v4l2_img_buf {
  uint8_t idx;            //< The index of the buffer
  size_t length;          //< The size of the buffer
  void *buf;              //< Pointer to the memory mapped buffer
  uint16_t w;             //< The width of the image
  uint16_t h;             //< The height of the image
};

#endif
