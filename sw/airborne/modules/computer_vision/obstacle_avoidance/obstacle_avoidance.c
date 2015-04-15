/*
 * Copyright (C) 2012-2015
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
 * @file obstacle_avoidance.h
 * 
 * Use color counting and opticflow codes to detect obstacles
 */

// Own header
#include "obstacle_avoidance.h"

#include <stdio.h> //printf
// Threaded computer vision
#include <pthread.h>

// Use stateGetNedToBodyEulers_f
#include "state.h"

// UDP RTP Images
#include "udp/socket.h"

// Video
#include "v4l/v4l2.h"

// Video Downlink options
//#define DOWNLINK_VIDEO 1         //to stream or not to stream
#define OPTICFLOW_SHOW_CORNERS 1 //to corner or not to corner
#define OPTICFLOW_SHOW_FLOW 1    //to flow or not to flow

// Downlink Video
#ifdef DOWNLINK_VIDEO
#include "encoding/jpeg.h"
#include "encoding/rtp.h"

#include <sys/time.h> //gettimeofday
#endif

// Sleep & Usleep functions
#include <unistd.h>

// Error Codes
#include "errno.h"

// Default broadcast IP
#ifndef VIDEO_SOCK_IP
#define VIDEO_SOCK_IP "192.168.1.255"
#endif

// I/O Sockets
#ifndef VIDEO_SOCK_OUT
#define VIDEO_SOCK_OUT 5000
#endif
#ifndef VIDEO_SOCK_IN
#define VIDEO_SOCK_IN 4999
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

/* Check setting for LK/FAST9 downsampling */
#ifndef LK_DOWNSIZE
#define LK_DOWNSIZE 4
#endif

/* The main opticflow variables */
static struct opticflow_t opticflow;                //< Opticflow calculations
static struct opticflow_result_t opticflow_result;  //< The opticflow result
static struct opticflow_state_t opticflow_state;    //< State of the drone to communicate with the opticflow
static struct v4l2_device *opticflow_dev;           //< The opticflow camera V4L2 device
static bool_t opticflow_got_result;                 //< When we have an optical flow calculation

/* The computer vision thread variables */
static pthread_t opticflow_calc_thread;             //< The optical flow calculation thread
static pthread_mutex_t opticflow_mutex;             //< Mutex lock for thread safety
volatile uint8_t opticflow_calc_thread_command = 0; //< Command to start/stop opticflow thread

/* Static functions */
static void *opticflow_module_calc(void *data);     //< The main optical flow calculation thread

/* Set color reference values (currently orange/red) */
uint8_t color_lum_min = 85;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 50;
uint8_t color_cb_max  = 120;
uint8_t color_cr_min  = 160;
uint8_t color_cr_max  = 240;
uint8_t color_downsize_factor = 2; //< Factor used to downsize image in color count function

// Set variable to communicate obstacle detection with waypoint navigation
int obstacleDetected = 0;

// Initialise module variables to 0
uint16_t color_counted = 0;
int IMU_init = 0;

/**
 * Initialize the optical flow module for the front camera
 */
void obstacle_avoidance_init(void)
{
  // Initialise the module variables to 0
  opticflow_state.psi = 0;

  // Initialize the opticflow calculation (for downsized image)
  opticflow_calc_init(&opticflow, 1280/LK_DOWNSIZE, 720/LK_DOWNSIZE); // 
  opticflow_got_result = FALSE;

  // Try to initialize the video device
  opticflow_dev = v4l2_init("/dev/video1", 1280, 720, 10);
  if (opticflow_dev == NULL) {
    printf("[opticflow_module] Could not initialize the %s V4L2 device.\n", "/dev/video1");
    return;
  }
}

/**
 * Update the optical flow state for the calculation thread
 * and update the stabilization loops with the newest result
 */
void obstacle_avoidance_run(void)
{
  pthread_mutex_lock(&opticflow_mutex);
  // Send Updated data to thread
  opticflow_state.psi = stateGetNedToBodyEulers_f()->psi;

  pthread_mutex_unlock(&opticflow_mutex);
}

/**
 * Start the optical flow calculation
 */
void obstacle_avoidance_start(void)
{
  // Check if we are not already running
  if(opticflow_calc_thread != 0) {
    printf("[opticflow_module] Opticflow already started!\n");
    return;
  }
  
  // Create the opticalflow calculation thread
  opticflow_calc_thread_command = 1;
  printf("[opticflow_module] Thread Started\n");
  int rc = pthread_create(&opticflow_calc_thread, NULL, opticflow_module_calc, NULL);
  if (rc) {
    printf("[opticflow_module] Could not initialize opticflow thread (return code: %d)\n", rc);
  }
}

/**
 * Stop the optical flow calculation
 */
void obstacle_avoidance_stop(void)
{
  // Stop the capturing
  v4l2_stop_capture(opticflow_dev);
  
  // Stop optic flow thread
  opticflow_calc_thread_command = 0;
  printf("[opticflow_module] Thread Closed\n");
}

/**
 * The main optical flow calculation thread
 * This thread passes the images trough the optical flow
 * calculator based on Lucas Kanade
 * The thread also performs color counting as backup for optical flow
 */
static void *opticflow_module_calc(void *data __attribute__((unused))) 
{
  // Start the streaming on the V4L2 device
  if(!v4l2_start_capture(opticflow_dev)) {
    printf("[opticflow_module] Could not start capture of the camera\n");
    return 0;
  }

#ifdef DOWNLINK_VIDEO
  // Create a new JPEG image
  struct image_t img_jpeg;
  image_create(&img_jpeg, opticflow_dev->w, opticflow_dev->h, IMAGE_JPEG);

  // Network Transmit
  struct UdpSocket *vsock;
  vsock = udp_socket(VIDEO_SOCK_IP, VIDEO_SOCK_OUT, VIDEO_SOCK_IN, FMS_BROADCAST);
#endif

  // Create a new downsampled image
  struct image_t small;
  image_create(&small, opticflow_dev->w/LK_DOWNSIZE, opticflow_dev->h/LK_DOWNSIZE, IMAGE_YUV422);

  // Set microsleep variable depending on VIDEO_FPS for usleep computation
  #ifdef DOWNLINK_VIDEO
  int microsleep = (int)(1000000. / VIDEO_FPS);
  struct timeval last_time;
  gettimeofday(&last_time, NULL);
  #endif

  /* Main loop of the optical flow calculation */
  while(opticflow_calc_thread_command > 0) {
    if (IMU_init == 0 && opticflow_state.psi != 0){
    IMU_init = 1;
    printf("[obstacle_avoidance] IMU initialisation complete");
    }

    #ifdef DOWNLINK_VIDEO
    struct timeval time;
    gettimeofday(&time, NULL);
    int dt = (int)(time.tv_sec - last_time.tv_sec) * 1000000 + (int)(time.tv_usec - last_time.tv_usec);
    if (dt < microsleep) { usleep(microsleep - dt); }
    last_time = time;
    #endif

    // Try to fetch an image
    struct image_t img;
    v4l2_image_get(opticflow_dev, &img);

    // Perform color count
    color_counted = color_count(&img, color_downsize_factor,
        color_lum_min,color_lum_max,
        color_cb_min,color_cb_max,
        color_cr_min,color_cr_max
        );

    // Downsample image
    image_yuv422_downsample(&img, &small, LK_DOWNSIZE);

    // Copy the state
    pthread_mutex_lock(&opticflow_mutex);
    struct opticflow_state_t temp_state;
    memcpy(&temp_state, &opticflow_state, sizeof(struct opticflow_state_t));
    pthread_mutex_unlock(&opticflow_mutex);

    // Do the optical flow calculation
    struct opticflow_result_t temp_result;
    opticflow_calc_frame(&opticflow, &temp_state, &small, &temp_result);

    // Copy the result if finished
    pthread_mutex_lock(&opticflow_mutex);
    memcpy(&opticflow_result, &temp_result, sizeof(struct opticflow_result_t));
    opticflow_got_result = TRUE;
    pthread_mutex_unlock(&opticflow_mutex);

    //printf("Opticflow got result = %d\n", opticflow_got_result);
    //printf("ColorCount = %d \n", color_counted);
 
    // Send results of color count and opticflow to waypoint input
    if ((opticflow_result.points[1] < -0.008 && opticflow_result.xdx_corr > 0.6) || (opticflow_result.points[1] > 0.008 && opticflow_result.xdx_corr > 0.6)) {
       obstacleDetected = 1; 
       printf("Flow detected ");
       printf("Results Points = %f with corr %f \n\n", opticflow_result.points[1], opticflow_result.xdx_corr);
       sleep(1);
    } else {if(color_counted > 8000) {
       obstacleDetected = 1;
       printf("Color detected ");
       printf("Colorcount = %d\n\n", color_counted);
       sleep(1);
    }}

#ifdef DOWNLINK_VIDEO
    // JPEG encode the image
    jpeg_encode_image(&small, &img_jpeg, VIDEO_QUALITY_FACTOR, FALSE);

    //Sending rtp frame
    send_rtp_frame(
        vsock,                           // UDP
        img_jpeg.buf, img_jpeg.buf_size, // JPEG
        img_jpeg.w, img_jpeg.h,          // Img Size
        0,                               // Format 422
        VIDEO_QUALITY_FACTOR,            // Jpeg-Quality
        0,                               // DRI Header
        0                                // 90kHz time increment
    );
    // Extra note: when the time increment is set to 0,
    // it is automaticaly calculated by the send_rtp_frame function
    // based on gettimeofday value. This seems to introduce some lag or jitter.
#endif

    // Free the image
    v4l2_image_free(opticflow_dev, &img);
  }
  // Free the downlink and small images
#ifdef DOWNLINK_VIDEO
  image_free(&img_jpeg);
#endif
  image_free(&small);
}
