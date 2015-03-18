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


// Own header
#include "modules/computer_vision/bottomcamsearch.h"

// UDP RTP Images
#include "modules/computer_vision/lib/udp/socket.h"

// Threaded computer vision
#include <pthread.h>

// Default broadcast IP
#ifndef VIDEO_SOCK_IP
#define VIDEO_SOCK_IP "192.168.1.255"
#endif

// Output socket can be defined from an offset
#ifdef VIDEO_SOCK_OUT_OFFSET
#define VIDEO_SOCK_OUT (5000+VIDEO_SOCK_OUT_OFFSET)
#endif

#ifndef VIDEO_SOCK_OUT
#define VIDEO_SOCK_OUT 5000
#endif

#ifndef VIDEO_SOCK_IN
#define VIDEO_SOCK_IN 4999
#endif

// Downsize factor for video stream
#ifndef VIDEO_DOWNSIZE_FACTOR
#define VIDEO_DOWNSIZE_FACTOR 4
#endif

// From 0 to 99 (99=high)
#ifndef VIDEO_QUALITY_FACTOR
#define VIDEO_QUALITY_FACTOR 50
#endif

// Frame Per Seconds
#ifndef VIDEO_FPS
#define VIDEO_FPS 4.
#endif

uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

int color_count = 0;

void bottomcamsearch_run(void) {
}

// take shot flag
int viewvideo_shot = 0;

int viewvideo_save_shot(void);
int viewvideo_save_shot(void)
{
  return 0;
}

/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

// Video
#include "modules/computer_vision/lib/v4l/video.h"
#include "modules/computer_vision/cv/resize.h"
#include "modules/computer_vision/cv/color.h"

#include "modules/computer_vision/cv/encoding/jpeg.h"
#include "modules/computer_vision/cv/encoding/rtp.h"

#include <stdio.h>
#include <string.h>

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void* data);
void *computervision_thread_main(void* data)
{
  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video1";
  vid.w=1280;
  vid.h=720;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }

  // Video Grabbing
  struct img_struct* img_new = video_create_image(&vid);

  // Video Resizing
  #define DOWNSIZE_FACTOR   2
  uint8_t quality_factor = 50; // From 0 to 99 (99=high)
  uint8_t dri_jpeg_header = 0;
  int millisleep = 250;

  struct img_struct small;
  small.w = vid.w / DOWNSIZE_FACTOR;
  small.h = vid.h / DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);


  // Video Compression
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);

  // Network Transmit
  struct UdpSocket* vsock;
  vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);

  while (computer_vision_thread_command > 0)
  {
    usleep(1000* millisleep);
    video_grab_image(&vid, img_new);

    // Resize: device by 4
    resize_uyuv(img_new, &small, DOWNSIZE_FACTOR);

    color_count = colorfilt_uyvy(&small,&small,
        color_lum_min,color_lum_max,
        color_cb_min,color_cb_max,
        color_cr_min,color_cr_max
        );

    printf("ColorCount = %d \n", color_count);

    // JPEG encode the image:
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
    uint32_t size = end-(jpegbuf);

    printf("Sending an image ...%u\n",size);
/*
 *
 */
    send_rtp_frame(
        vsock,            // UDP
        jpegbuf,size,     // JPEG
        small.w, small.h, // Img Size
        0,                // Format 422
        quality_factor,               // Jpeg-Quality
        dri_jpeg_header,                // DRI Header
        0              // 90kHz time increment
     );
  }
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;
}

void bottomcamsearch_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void bottomcamsearch_stop(void)
{
  computer_vision_thread_command = 0;
}



