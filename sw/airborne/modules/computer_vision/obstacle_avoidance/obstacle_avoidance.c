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
#include "obstacle_avoidance.h"

// Sockets
#include <stdio.h> //printf
#include <string.h> //malloc
#include <unistd.h> //usleep
#include <sys/time.h> //gettimeofday

// UDP RTP Images
#include "udp/socket.h"

// Video
#include "v4l/v4l2.h"
#include "resize.h"
#include "color.h"

// Downlink Video
/*#define DOWNLINK_VIDEO 1*/
#ifdef DOWNLINK_VIDEO
#include "encoding/jpeg.h"
#include "encoding/rtp.h"
#endif

#define DEBUG_INFO(X, ...) ;

// Threaded computer vision
#include <pthread.h>

// Computervision runs in a thread
/*#include "inter_thread_data.h"*/

// Default broadcast IP
#ifndef VIDEO_SOCK_IP
#define VIDEO_SOCK_IP "192.168.1.255"
#endif

// I/O Sockets
#ifndef VIDEO_SOCK_OUT
#define VIDEO_SOCK_OUT 5000
#endif

#ifndef VIDEO_SOCK_IN
#define VIDEO_SOCK_IN 5001
#endif

// Downsize factor for video stream
#ifndef VIDEO_DOWNSIZE_FACTOR
#define VIDEO_DOWNSIZE_FACTOR 2
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

void obstacle_avoidance_run(void) {
}

/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void* data);
void *computervision_thread_main(void* data)
{
/*  // Local data in/out*/
/*  struct CVresults vision_results;*/
/*  struct PPRZinfo autopilot_data;*/

  // Create V4L2 device video1 = front camera
  struct v4l2_device *dev = v4l2_init("/dev/video1", 1280, 720, 4);
  if (dev == NULL) {
    printf("Error initialising video\n");
    return 0;
  }

  // Start the streaming on the V4L2 device
  if(!v4l2_start_capture(dev)) {
    printf("Could not start capture\n");
    return 0;
  }

  // Video Resizing
  struct v4l2_img_buf small;
  small.w = dev->w / VIDEO_DOWNSIZE_FACTOR;
  small.h = dev->h / VIDEO_DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);

  #ifdef DOWNLINK_VIDEO
  // Video Compression
  uint8_t *jpegbuf = (uint8_t*)malloc(dev->w*dev->h*2);

  // Network Transmit
  struct UdpSocket *vsock;
  vsock = udp_socket(VIDEO_SOCK_IP, VIDEO_SOCK_OUT, VIDEO_SOCK_IN, FMS_BROADCAST);
  #endif

  // time
  int microsleep = (int)(1000000. / VIDEO_FPS);
  struct timeval last_time;
  gettimeofday(&last_time, NULL);

  while (computer_vision_thread_command > 0) {
    // compute usleep to have a more stable frame rate
    struct timeval time;
    gettimeofday(&time, NULL);
    int dt = (int)(time.tv_sec - last_time.tv_sec) * 1000000 + (int)(time.tv_usec - last_time.tv_usec);
    if (dt < microsleep) { usleep(microsleep - dt); }
    last_time = time;

    // Wait for a new frame
    struct v4l2_img_buf *img = v4l2_image_get(dev);
    printf("Got a new frame");

    // Resize: device by VIDEO_DOWNSIZE_FACTOR
    resize_uyuv(img, &small, VIDEO_DOWNSIZE_FACTOR);

    color_count = colorfilt_uyvy(&small,&small, 
        color_lum_min,color_lum_max,
        color_cb_min,color_cb_max,
        color_cr_min,color_cr_max
        );

    printf("ColorCount = %d \n", color_count);

    #ifdef DOWNLINK_VIDEO
    // JPEG encode the image:
    uint8_t quality_factor = VIDEO_QUALITY_FACTOR; // From 0 to 99 (99=high)
    uint8_t dri_jpeg_header = 0;
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
    uint32_t size = end-(jpegbuf);

/*    printf("Sending an image ...%u\n",size);*/

    //Sending rtp frame
    send_rtp_frame(
        vsock,            // UDP
        jpegbuf,size,     // JPEG
        small.w, small.h, // Img Size
        0,                // Format 422
        quality_factor,   // Jpeg-Quality
        dri_jpeg_header,  // DRI Header
        0                 // 90kHz time increment
     );
    // Extra note: when the time increment is set to 0,
    // it is automaticaly calculated by the send_rtp_frame function
    // based on gettimeofday value. This seems to introduce some lag or jitter.
    // An other way is to compute the time increment and set the correct value.
    // It seems that a lower value is also working (when the frame is received
    // the timestamp is always "late" so the frame is displayed immediately).
    // Here, we set the time increment to the lowest possible value
    // (1 = 1/90000 s) which is probably stupid but is actually working.
    #endif
  
  // Free the image
  v4l2_image_free(dev, img);
  }

  printf("Thread Closed\n");
  v4l2_close(dev);
  computervision_thread_status = -100;
  return 0;
}

void obstacle_avoidance_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void obstacle_avoidance_stop(void)
{
  computer_vision_thread_command = 0;
}



