/*
 * Copyright (C) 2012-2014 The Paparazzi Community
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include "modules/computer_vision/viewvideo.h"
#include "modules/computer_vision/cv.h"
#include "generated/airframe.h"

#if defined BOARD_BEBOP && !defined USE_H264
#define USE_H264 FALSE
#endif
PRINT_CONFIG_VAR(USE_H264)

#if USE_H264
#ifndef VIEWVIDEO_H264_BITRATE
#define VIEWVIDEO_H264_BITRATE 0.5
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_H264_BITRATE)
#endif


#if defined BOARD_BEBOP && !defined USE_OPENGL
#define USE_OPENGL FALSE
#endif
PRINT_CONFIG_VAR(USE_OPENGL)

#if USE_H264
#include "modules/computer_vision/lib/encoding/P7_H264.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"
#include "generated/airframe.h"

#if USE_OPENGL
#include "modules/computer_vision/lib/opengl/opengl.h"
#endif

#include BOARD_CONFIG

// Downsize factor for video stream
#ifndef VIEWVIDEO_DOWNSIZE_FACTOR
#define VIEWVIDEO_DOWNSIZE_FACTOR 1
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DOWNSIZE_FACTOR)

// From 0 to 99 (99=high)
#ifndef VIEWVIDEO_QUALITY_FACTOR
#define VIEWVIDEO_QUALITY_FACTOR 50
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_QUALITY_FACTOR)

// RTP time increment at 90kHz (default: 0 for automatic)
#ifndef VIEWVIDEO_RTP_TIME_INC
#define VIEWVIDEO_RTP_TIME_INC 0
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_RTP_TIME_INC)

// Define stream framerate
#ifndef VIEWVIDEO_FPS
#define VIEWVIDEO_FPS 5
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_FPS)

// Default image folder
#ifndef VIEWVIDEO_SHOT_PATH
#ifdef VIDEO_THREAD_SHOT_PATH
#define VIEWVIDEO_SHOT_PATH VIDEO_THREAD_SHOT_PATH
#else
#ifdef BOARD_BEBOP
#define VIEWVIDEO_SHOT_PATH /data/ftp/internal_000
#else
#define VIEWVIDEO_SHOT_PATH /data/video/images
#endif
#endif
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_SHOT_PATH)

// Define stream priority
#ifndef VIEWVIDEO_NICE_LEVEL
#define VIEWVIDEO_NICE_LEVEL 5
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_NICE_LEVEL)

// Check if we are using netcat instead of RTP/UDP
#ifndef VIEWVIDEO_USE_NETCAT
#define VIEWVIDEO_USE_NETCAT FALSE
#endif

#if !VIEWVIDEO_USE_NETCAT && !(defined VIEWVIDEO_USE_RTP)
#define VIEWVIDEO_USE_RTP TRUE
#endif

#if VIEWVIDEO_USE_NETCAT
#include <sys/wait.h>
PRINT_CONFIG_MSG("[viewvideo] Using netcat.")
#else
struct UdpSocket video_sock1;
struct UdpSocket video_sock2;
PRINT_CONFIG_MSG("[viewvideo] Using RTP/UDP stream.")
PRINT_CONFIG_VAR(VIEWVIDEO_USE_RTP)
#endif

#ifndef VIEWVIDEO_VERBOSE
#define VIEWVIDEO_VERBOSE 0
#endif

#ifndef VIEWVIDEO_WRITE_VIDEO
#define VIEWVIDEO_WRITE_VIDEO 0
#endif

#ifndef VIEWVIDEO_STREAM
#define VIEWVIDEO_STREAM 1
#endif

#ifndef VIEWVIDEO_VIDEO_FILE
#define VIEWVIDEO_VIDEO_FILE video_file
#endif

#ifndef VIEWVIDEO_DATETIME_NAME
#define VIEWVIDEO_DATETIME_NAME 1
#endif

#if VIEWVIDEO_DATETIME_NAME
#include <time.h>
#endif

#define printf_debug    if(VIEWVIDEO_VERBOSE > 0) printf

/* These are defined with configure */
PRINT_CONFIG_VAR(VIEWVIDEO_CAMERA)
PRINT_CONFIG_VAR(VIEWVIDEO_CAMERA2)
PRINT_CONFIG_VAR(VIEWVIDEO_HOST)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT_OUT)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT2_OUT)

// Initialize the viewvideo structure with the defaults
struct viewvideo_t viewvideo = {
  .is_streaming = FALSE,
  .downsize_factor = VIEWVIDEO_DOWNSIZE_FACTOR,
  .quality_factor = VIEWVIDEO_QUALITY_FACTOR,
#if !VIEWVIDEO_USE_NETCAT
  .use_rtp = VIEWVIDEO_USE_RTP,
#endif
};

#if USE_H264
static P7_H264_context_t videoEncoder;
#endif
static FILE *video_file;
bool viewvideo_recording = FALSE;

/**
 * Handles all the video streaming and saving of the image shots
 * This is a separate thread, so it needs to be thread safe!
 */
#if !USE_H264 || VIEWVIDEO_CAMERA2
static struct image_t *viewvideo_function(struct UdpSocket *socket, struct image_t *img, uint32_t *rtp_frame_nr)
{
  // Resize image if needed
  struct image_t img_small;
  image_create(&img_small,
               img->w / viewvideo.downsize_factor,
               img->h / viewvideo.downsize_factor,
               IMAGE_YUV422);

  // Create the JPEG encoded image
  struct image_t img_jpeg;
  image_create(&img_jpeg, img_small.w, img_small.h, IMAGE_JPEG);

#if VIEWVIDEO_USE_NETCAT
  char nc_cmd[64];
  sprintf(nc_cmd, "nc %s %d 2>/dev/null", STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT);
#endif

  if (viewvideo.is_streaming) {

    // Only resize when needed
    if (viewvideo.downsize_factor != 1) {
      image_yuv422_downsample(img, &img_small, viewvideo.downsize_factor);
      jpeg_encode_image(&img_small, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
    } else {
      jpeg_encode_image(img, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
    }

#if VIEWVIDEO_USE_NETCAT
    // Open process to send using netcat (in a fork because sometimes kills itself???)
    pid_t pid = fork();

    if (pid < 0) {
      printf("[viewvideo] Could not create netcat fork.\n");
    } else if (pid == 0) {
      // We are the child and want to send the image
      FILE *netcat = popen(nc_cmd, "w");
      if (netcat != NULL) {
        fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, netcat);
        pclose(netcat); // Ignore output, because it is too much when not connected
      } else {
        printf("[viewvideo] Failed to open netcat process.\n");
      }

      // Exit the program since we don't want to continue after transmitting
      exit(0);
    } else {
      // We want to wait until the child is finished
      wait(NULL);
    }
#else
    if (viewvideo.use_rtp) {
      // Send image with RTP
      rtp_frame_send(
        socket,              // UDP socket
        &img_jpeg,
        0,                        // Format 422
        VIEWVIDEO_QUALITY_FACTOR, // Jpeg-Quality
        0,                        // DRI Header
        (img->ts.tv_sec * 1000000 + img->ts.tv_usec),
        rtp_frame_nr
      );
    }
#endif
  }

  // Free all buffers
  image_free(&img_jpeg);
  image_free(&img_small);
  return NULL; // No new images were created
}
#endif

#if USE_H264
static struct image_t *viewvideo_function_h264(struct UdpSocket *socket, struct image_t *img)
{
  int32_t h264BufferIndex, size;
  uint8_t* h264Buffer;
  struct image_t releaseImg;
  if(viewvideo_recording && video_file == NULL){
    viewvideo_start_recording();
  }
  if(!viewvideo_recording && video_file != NULL){
    viewvideo_stop_recording();
  }
  if (viewvideo.is_streaming) {
    P7_H264_releaseInputBuffer(&videoEncoder, img->buf_idx);

    while ((h264BufferIndex = P7_H264_find_FreeBuffer(videoEncoder.inputBuffers, BUFFER_TOBE_RELEASED, videoEncoder.numInputBuffers)) != -1){
      releaseImg.buf_idx = h264BufferIndex;
      videoEncoder.inputBuffers[h264BufferIndex].status = BUFFER_FREE;
      v4l2_image_free(VIEWVIDEO_CAMERA.thread.dev, &releaseImg);
    }

    while (!P7_H264_getOutputBuffer(&videoEncoder, &h264BufferIndex))
    {

      h264Buffer = P7_H264_bufferIndex2OutputPointer(&videoEncoder, h264BufferIndex);
      size = P7_H264_bufferIndex2OutputSize(&videoEncoder, h264BufferIndex);


      if (size == 0)
        ;//fprintf(stderr, "%s:%d warning, no data to write\n",__FILE__,__LINE__);
      else {
        printf_debug("Got frame of size: %d\r\n", size);
        printf_debug("Byte: %2X %2X %2X %2X %2X\n", h264Buffer[0],h264Buffer[1], h264Buffer[2], h264Buffer[3], h264Buffer[4]);
#if VIEWVIDEO_STREAM
        rtp_frame_send_h264(socket, h264Buffer, size);
#endif

        if(viewvideo_recording && video_file != NULL){
            fwrite(h264Buffer, size, 1, video_file);
        }
      }
      P7_H264_releaseOutputBuffer(&videoEncoder, h264BufferIndex);
    }
  } else {
    v4l2_image_free(VIEWVIDEO_CAMERA.thread.dev, img);
  }
  return NULL; // No new images were created
}
#endif


#ifdef VIEWVIDEO_CAMERA
#if USE_H264
static struct image_t *viewvideo_function1(struct image_t *img)
{
  return viewvideo_function_h264(&video_sock1, img);
}
#else
static struct image_t *viewvideo_function1(struct image_t *img)
{
  static uint32_t rtp_frame_nr = 0;
  return viewvideo_function(&video_sock1, img, &rtp_frame_nr);
}
#endif
#endif

#ifdef VIEWVIDEO_CAMERA2
static struct image_t *viewvideo_function2(struct image_t *img)
{
  static uint32_t rtp_frame_nr = 0;
  return viewvideo_function(&video_sock2, img, &rtp_frame_nr);
}
#endif

/**
 * Initialize the view video
 */
void viewvideo_init(void)
{
  viewvideo.is_streaming = VIEWVIDEO_STREAM;
#if VIEWVIDEO_USE_NETCAT
  // Create an Netcat receiver file for the streaming
  sprintf(save_name, "%s/netcat-recv.sh", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "i=0\n");
    fprintf(fp, "while true\n");
    fprintf(fp, "do\n");
    fprintf(fp, "\tn=$(printf \"%%04d\" $i)\n");
    fprintf(fp, "\tnc -l 0.0.0.0 %d > img_${n}.jpg\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "\ti=$((i+1))\n");
    fprintf(fp, "done\n");
    fclose(fp);
  } else {
    printf("[viewvideo] Failed to create netcat receiver file.\n");
  }
#else
#ifdef VIEWVIDEO_CAMERA
#if !USE_H264
  struct video_listener *listener1 = cv_add_to_device_async(&VIEWVIDEO_CAMERA, viewvideo_function1,
                                     VIEWVIDEO_NICE_LEVEL);
  listener1->maximum_fps = VIEWVIDEO_FPS;
  fprintf(stderr, "[viewvideo] Added asynchronous video streamer lister for CAMERA1\n");
#else
  struct video_listener *listener1 = cv_add_to_device(&VIEWVIDEO_CAMERA, viewvideo_function1);
  listener1->maximum_fps = 0;
  fprintf(stderr, "[viewvideo] Added synchronous video streamer lister for CAMERA1\n");
#endif
#endif

#ifdef VIEWVIDEO_CAMERA2
  struct video_listener *listener2 = cv_add_to_device(&VIEWVIDEO_CAMERA2, viewvideo_function2);
  listener2->maximum_fps = 0;
  fprintf(stderr, "[viewvideo] Added synchronous video streamer lister for CAMERA2\n");
#endif

#if USE_H264
  videoEncoder.inputType = H264ENC_YUV422_INTERLEAVED_UYVY;
  videoEncoder.bitRate   = (uint32_t) (VIEWVIDEO_H264_BITRATE * 1024 * 1024); // in Mbps
  videoEncoder.frameRate = VIEWVIDEO_FPS;
  videoEncoder.intraRate = 0;//VIEWVIDEO_FPS;
  P7_H264_open(&videoEncoder, VIEWVIDEO_CAMERA.thread.dev);
  char save_name[512];
  // Create an SDP file for the streaming
  sprintf(save_name, "%s/stream.sdp", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "v=0\n");
    fprintf(fp, "m=video %d RTP/AVP 96\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "a=rtpmap:96 H264\n");
    fprintf(fp, "a=framerate:%d\n", (int)(VIEWVIDEO_FPS));
    fprintf(fp, "c=IN IP4 0.0.0.0\n");
    fclose(fp);
  } else {
    printf_debug("[viewvideo] Failed to create SDP file.\n");
  }
#ifdef VIEWVIDEO_CAMERA2
  char save_name2[512];
  // Create an SDP file for the streaming
  sprintf(save_name2, "%s/stream2.sdp", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp2 = fopen(save_name2, "w");
  if (fp2 != NULL) {
    fprintf(fp2, "v=0\n");
    fprintf(fp2, "m=video %d RTP/AVP 26\n", (int)(VIEWVIDEO_PORT2_OUT));
    fprintf(fp2, "c=IN IP4 0.0.0.0\n");
    fclose(fp2);
  } else {
    printf_debug("[viewvideo] Failed to create stream #2 SDP file.\n");
  }
#endif
#endif
  // Open udp socket
#ifdef VIEWVIDEO_CAMERA
  if (udp_socket_create(&video_sock1, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST)) {
#if USE_H264
    udp_socket_set_sendbuf(&video_sock1, 1042 * 1024);
#endif
    printf("[viewvideo]: failed to open view video socket, HOST=%s, port=%d\n", STRINGIFY(VIEWVIDEO_HOST),
           VIEWVIDEO_PORT_OUT);
  }
#endif

#ifdef VIEWVIDEO_CAMERA2
  if (udp_socket_create(&video_sock2, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT2_OUT, -1, VIEWVIDEO_BROADCAST)) {
    udp_socket_set_sendbuf(&video_sock1, 1042 * 1024);
    printf("[viewvideo]: failed to open view video socket, HOST=%s, port=%d\n", STRINGIFY(VIEWVIDEO_HOST),
           VIEWVIDEO_PORT2_OUT);
  }
#endif
#endif
}

/*
 * Start / Stop recording function of H264 stream
 */

void viewvideo_start_recording( void ){
    char video_name[512];
    uint32_t counter = 0;
    // Check for available files
#if VIEWVIDEO_DATETIME_NAME
  time_t timer;
  char date_buffer[26];
  struct tm* tm_info;
  timer = time(NULL);
  tm_info = localtime(&timer);
  strftime(date_buffer, 26, "%Y_%m_%d__%H_%M_%S", tm_info);
  sprintf(video_name, "%s/%s_%s_%s_%05d.h264", STRINGIFY(VIEWVIDEO_SHOT_PATH), AIRFRAME_NAME, STRINGIFY(VIEWVIDEO_VIDEO_FILE), date_buffer, counter);
  while ((video_file = fopen(video_name, "r"))) {
    fclose(video_file);
    counter++;
    sprintf(video_name, "%s/%s_%s_%s_%05d.h264", STRINGIFY(VIEWVIDEO_SHOT_PATH), AIRFRAME_NAME, STRINGIFY(VIEWVIDEO_VIDEO_FILE), date_buffer, counter);
  }
#else
  sprintf(video_name, "%s/%s_%s.h264", STRINGIFY(VIEWVIDEO_SHOT_PATH), AIRFRAME_NAME, STRINGIFY(VIEWVIDEO_VIDEO_FILE));
  while ((video_file = fopen(video_name, "r"))) {
    fclose(video_file);
    counter++;
    sprintf(video_name, "%s/%s_%s_%05d.h264", STRINGIFY(VIEWVIDEO_SHOT_PATH), AIRFRAME_NAME, STRINGIFY(VIEWVIDEO_VIDEO_FILE), counter);
  }
#endif
    int tries       = 0;
    int maxtries    = 5;
    do {
        video_file = fopen(video_name, "w");
        tries++;
    } while(video_file == NULL && tries < maxtries);
    if(video_file == NULL)
    {
        printf("[viewvideo] Failed to create .h264 file.\n");
        viewvideo_recording = false;
    }
    else{
        viewvideo_recording = true;
#if !VIEWVIDEO_STREAM
        viewvideo.is_streaming = true;
#endif
    }
}

void viewvideo_stop_recording( void ){
    if(video_file){
        fclose(video_file);
    }
    video_file          = NULL;
    viewvideo_recording = false;
#if !VIEWVIDEO_STREAM
        viewvideo.is_streaming = false;
#endif
}
