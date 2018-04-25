/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2016 Kimberly McGuire <k.n.mcguire@tudelft.nl
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
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"
#include "lib/vision/act_fast.h"
#include "lib/vision/edge_flow.h"
#include "size_divergence.h"
#include "linear_flow_fit.h"

// Kalman filter
#include "lib/filters/kalman_filter_vision.h"
#include "subsystems/imu.h"

// for measuring time
#include "mcu_periph/sys_time.h"


// LOGGING:
// for determining when to log vectors and images:
#include "modules/ctrl/optical_flow_landing.h"
float previous_log_time;
/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/ftp/internal_000/
#endif
/** The file pointer */
static FILE *ofc_file_logger = NULL;
#include "modules/computer_vision/video_capture.h"
#define LOG_DEROTATION true
#define LOG_SSL false


// whether to show the flow and corners:
#define OPTICFLOW_SHOW_FLOW 0
#define OPTICFLOW_SHOW_CORNERS 0
#define OPTICFLOW_SHOW_INLIERS 0

#define EXHAUSTIVE_FAST 0
#define ACT_FAST 1
uint16_t n_time_steps = 10;
uint16_t n_agents = 25;
// corner method:
#define CORNER_METHOD 1

// YUV histograms, number of bins:
#define DOWNSELECT_VECTORS 0
#define N_BINS_UV 5
// Number of cells in image:
#define N_CELLS 3

// What methods are run to determine divergence, lateral flow, etc.
// SIZE_DIV looks at line sizes and only calculates divergence
#define SIZE_DIV 0
// LINEAR_FIT makes a linear optical flow field fit and extracts a lot of information:
// relative velocities in x, y, z (divergence / time to contact), the slope of the surface, and the surface roughness.
#define LINEAR_FIT 0

// Camera parameters (defaults are from an ARDrone 2)
#ifndef OPTICFLOW_FOV_W
#define OPTICFLOW_FOV_W 0.89360857702
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FOV_W)

#ifndef OPTICFLOW_FOV_H
#define OPTICFLOW_FOV_H 0.67020643276
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FOV_H)

#ifndef OPTICFLOW_FX
#define OPTICFLOW_FX 343.1211
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FX)

#ifndef OPTICFLOW_FY
#define OPTICFLOW_FY 348.5053
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FY)

/* Set the default values */
#ifndef OPTICFLOW_MAX_TRACK_CORNERS
#define OPTICFLOW_MAX_TRACK_CORNERS 25
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_TRACK_CORNERS)

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_WINDOW_SIZE)

#ifndef OPTICFLOW_SEARCH_DISTANCE
#define OPTICFLOW_SEARCH_DISTANCE 20
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SEARCH_DISTANCE)

#ifndef OPTICFLOW_SUBPIXEL_FACTOR
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SUBPIXEL_FACTOR)

#ifndef OPTICFLOW_RESOLUTION_FACTOR
#define OPTICFLOW_RESOLUTION_FACTOR 100
#endif
PRINT_CONFIG_VAR(OPTICFLOW_RESOLUTION_FACTOR)

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_ITERATIONS)

#ifndef OPTICFLOW_THRESHOLD_VEC
#define OPTICFLOW_THRESHOLD_VEC 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_THRESHOLD_VEC)

#ifndef OPTICFLOW_PYRAMID_LEVEL
#define OPTICFLOW_PYRAMID_LEVEL 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_PYRAMID_LEVEL)

#ifndef OPTICFLOW_FAST9_ADAPTIVE
#define OPTICFLOW_FAST9_ADAPTIVE TRUE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_ADAPTIVE)

#ifndef OPTICFLOW_FAST9_THRESHOLD
#define OPTICFLOW_FAST9_THRESHOLD 20
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_THRESHOLD)

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE
#define OPTICFLOW_FAST9_MIN_DISTANCE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_MIN_DISTANCE)

#ifndef OPTICFLOW_FAST9_PADDING
#define OPTICFLOW_FAST9_PADDING 20
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_PADDING)

// thresholds FAST9 that are currently not set from the GCS:
#define FAST9_LOW_THRESHOLD 5
#define FAST9_HIGH_THRESHOLD 60

#ifndef OPTICFLOW_METHOD
#define OPTICFLOW_METHOD 0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_METHOD)

#if OPTICFLOW_METHOD > 1
#error WARNING: Both Lukas Kanade and EdgeFlow are NOT selected
#endif

#ifndef OPTICFLOW_DEROTATION
#define OPTICFLOW_DEROTATION TRUE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_DEROTATION)

#ifndef OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X
#define OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X 1.0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X)

#ifndef OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y
#define OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y 1.0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y)

#ifndef OPTICFLOW_MEDIAN_FILTER
#define OPTICFLOW_MEDIAN_FILTER FALSE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MEDIAN_FILTER)

#ifndef OPTICFLOW_KALMAN_FILTER
#define OPTICFLOW_KALMAN_FILTER TRUE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_KALMAN_FILTER)

#ifndef OPTICFLOW_KALMAN_FILTER_PROCESS_NOISE
#define OPTICFLOW_KALMAN_FILTER_PROCESS_NOISE 0.01
#endif
PRINT_CONFIG_VAR(OPTICFLOW_KALMAN_FILTER_PROCESS_NOISE)

#ifndef OPTICFLOW_FEATURE_MANAGEMENT
#define OPTICFLOW_FEATURE_MANAGEMENT 1
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FEATURE_MANAGEMENT)

#ifndef OPTICFLOW_FAST9_REGION_DETECT
#define OPTICFLOW_FAST9_REGION_DETECT 1
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_REGION_DETECT)

#ifndef OPTICFLOW_FAST9_NUM_REGIONS
#define OPTICFLOW_FAST9_NUM_REGIONS 9
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_NUM_REGIONS)

#ifndef OPTICFLOW_COL_SAMPLES
#define OPTICFLOW_COL_SAMPLES 250
#endif
PRINT_CONFIG_VAR(OPTICFLOW_COL_SAMPLES)

#ifndef OPTICFLOW_COL_THRESH
#define OPTICFLOW_COL_THRESH 40
#endif
PRINT_CONFIG_VAR(OPTICFLOW_COL_THRESH)

#ifndef OPTICFLOW_COL_MIN_UV
#define OPTICFLOW_COL_MIN_UV 90
#endif
PRINT_CONFIG_VAR(OPTICFLOW_COL_MIN_UV)

#ifndef OPTICFLOW_COL_MAX_UV
#define OPTICFLOW_COL_MAX_UV 164
#endif
PRINT_CONFIG_VAR(OPTICFLOW_COL_MIN_UV)

//Include median filter
#include "filters/median_filter.h"
struct MedianFilterInt vel_x_filt, vel_y_filt;


/* Functions only used here */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);
static int cmp_flow(const void *a, const void *b);
static int cmp_array(const void *a, const void *b);



/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 */
void opticflow_calc_init(struct opticflow_t *opticflow)
{
  /* Set the default values */
  opticflow->method = OPTICFLOW_METHOD; //0 = LK_fast9, 1 = Edgeflow
  opticflow->window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow->search_distance = OPTICFLOW_SEARCH_DISTANCE;
  opticflow->derotation = OPTICFLOW_DEROTATION; //0 = OFF, 1 = ON
  opticflow->derotation_correction_factor_x = OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X;
  opticflow->derotation_correction_factor_y = OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y;

  opticflow->max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
  opticflow->subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
  opticflow->resolution_factor = OPTICFLOW_RESOLUTION_FACTOR;
  opticflow->max_iterations = OPTICFLOW_MAX_ITERATIONS;
  opticflow->threshold_vec = OPTICFLOW_THRESHOLD_VEC;
  opticflow->pyramid_level = OPTICFLOW_PYRAMID_LEVEL;
  opticflow->median_filter = OPTICFLOW_MEDIAN_FILTER;
  opticflow->kalman_filter = OPTICFLOW_KALMAN_FILTER;
  opticflow->kalman_filter_process_noise = OPTICFLOW_KALMAN_FILTER_PROCESS_NOISE;
  opticflow->feature_management = OPTICFLOW_FEATURE_MANAGEMENT;
  opticflow->fast9_region_detect = OPTICFLOW_FAST9_REGION_DETECT;
  opticflow->fast9_num_regions = OPTICFLOW_FAST9_NUM_REGIONS;

  opticflow->fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
  opticflow->fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
  opticflow->fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;
  opticflow->fast9_padding = OPTICFLOW_FAST9_PADDING;
  opticflow->fast9_rsize = 512;
  opticflow->fast9_ret_corners = malloc(sizeof(struct point_t) * opticflow->fast9_rsize);

  opticflow->n_samples_color_histogram = OPTICFLOW_COL_SAMPLES;
  opticflow->color_similarity_threshold = OPTICFLOW_COL_THRESH;
  opticflow->color_min_UV = OPTICFLOW_COL_MIN_UV;
  opticflow->color_max_UV = OPTICFLOW_COL_MAX_UV;

  // prepare logging:
  previous_log_time = get_sys_time_float();

  char filename[512];
    // Check for available files
    sprintf(filename, "%s/optical_flow.csv", STRINGIFY(FILE_LOGGER_PATH));
    if (!(ofc_file_logger = fopen(filename, "r"))) {
        // we have to create the file:
        ofc_file_logger = fopen(filename, "w");

        if (ofc_file_logger != NULL) {
            if(LOG_DEROTATION) {
                fprintf(ofc_file_logger, "delta_phi,delta_theta,delta_psi,flow_x,flow_y,der_flow_x,der_flow_y");
            }
            else {
                fprintf(ofc_file_logger, "cov_div,pused,n_vectors");
                for(int i = 0; i < OPTICFLOW_MAX_TRACK_CORNERS; i++) {
                    fprintf(ofc_file_logger, ",px%d,py%d,fx%d,fy%d",i,i,i,i);
                }
            }

            fprintf(ofc_file_logger, "\n");
        }
    }
    else {
        ofc_file_logger = fopen(filename, "a");
    }
}
/**
 * Run the optical flow with fast9 and lukaskanade on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
void calc_fast9_lukas_kanade(struct opticflow_t *opticflow, struct opticflow_state_t *cam_state, struct image_t *img,
                             struct opticflow_result_t *result)
{
  if (opticflow->just_switched_method) {
    // Create the image buffers
    image_create(&opticflow->img_gray, img->w, img->h, IMAGE_GRAYSCALE);
    image_create(&opticflow->prev_img_gray, img->w, img->h, IMAGE_GRAYSCALE);

    // Set the previous values
    opticflow->got_first_img = false;
    FLOAT_RATES_ZERO(opticflow->prev_rates);

    // Init median filters with zeros
    init_median_filter_i(&vel_x_filt, MEDIAN_DEFAULT_SIZE);
    init_median_filter_i(&vel_y_filt, MEDIAN_DEFAULT_SIZE);
  }

  // variables for size_divergence:
  float size_divergence; int n_samples;

  // variables for linear flow fit:
  float error_threshold;
  int n_iterations_RANSAC, n_samples_RANSAC, success_fit;
  struct linear_flow_fit_info fit_info;

  // Update FPS for information
  result->fps = 1 / (timeval_diff(&opticflow->prev_timestamp, &img->ts) / 1000.);
  opticflow->prev_timestamp = img->ts;

  // Convert image to grayscale
  image_to_grayscale(img, &opticflow->img_gray);

  // Copy to previous image if not set
  if (!opticflow->got_first_img) {
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    opticflow->got_first_img = true;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // if feature_management is selected and tracked corners drop below a threshold, redetect
  if ((opticflow->feature_management) && (result->corner_cnt < opticflow->max_track_corners / 2)) {
    // no need for "per region" re-detection when there are no previous corners
    if ((!opticflow->fast9_region_detect) || (result->corner_cnt == 0)) {
      fast9_detect(&opticflow->prev_img_gray, opticflow->fast9_threshold, opticflow->fast9_min_distance,
                   opticflow->fast9_padding, opticflow->fast9_padding, &result->corner_cnt,
                   &opticflow->fast9_rsize,
                   &opticflow->fast9_ret_corners,
                   NULL);
    } else {
      // allocating memory and initializing the 2d array that holds the number of corners per region and its index (for the sorting)
      uint16_t **region_count = malloc(opticflow->fast9_num_regions * sizeof(uint16_t *));
      for (uint16_t i = 0; i < opticflow->fast9_num_regions ; i++) {
        region_count[i] = malloc(sizeof(uint16_t) * 2);
        region_count[i][0] = 0;
        region_count[i][1] = i;
      }
      for (uint16_t i = 0; i < result->corner_cnt; i++) {
        region_count[(opticflow->fast9_ret_corners[i].x / (img->w / (uint8_t)sqrt(opticflow->fast9_num_regions))
                      + opticflow->fast9_ret_corners[i].y / (img->h / (uint8_t)sqrt(opticflow->fast9_num_regions)) * (uint8_t)sqrt(opticflow->fast9_num_regions))][0]++;
      }

      //sorting region_count array according to first column (number of corners).
      qsort(region_count, opticflow->fast9_num_regions, sizeof(region_count[0]), cmp_array);

      // Detecting corners from the region with the less to the one with the most, until a desired total is reached.
      for (uint16_t i = 0; i < opticflow->fast9_num_regions && result->corner_cnt < 2 * opticflow->max_track_corners ; i++) {

        // Find the boundaries of the region of interest
        uint16_t *roi = malloc(4 * sizeof(uint16_t));
        roi[0] = (region_count[i][1] % (uint8_t)sqrt(opticflow->fast9_num_regions)) * (img->w / (uint8_t)sqrt(opticflow->fast9_num_regions));
        roi[1] = (region_count[i][1] / (uint8_t)sqrt(opticflow->fast9_num_regions)) * (img->h / (uint8_t)sqrt(opticflow->fast9_num_regions));
        roi[2] = roi[0] + (img->w / (uint8_t)sqrt(opticflow->fast9_num_regions));
        roi[3] = roi[1] + (img->h / (uint8_t)sqrt(opticflow->fast9_num_regions));

        fast9_detect(&opticflow->prev_img_gray, opticflow->fast9_threshold, opticflow->fast9_min_distance,
                     opticflow->fast9_padding, opticflow->fast9_padding, &result->corner_cnt,
                     &opticflow->fast9_rsize,
                     &opticflow->fast9_ret_corners,
                     roi);
        free(roi);
      }
      for (uint16_t i = 0; i < opticflow->fast9_num_regions; i++) {
        free(region_count[i]);
      }
      free(region_count);
    }
  } else if (!opticflow->feature_management) {
    // needs to be set to 0 because result is now static
    result->corner_cnt = 0;


    float pre_corner = get_sys_time_float();

    if(CORNER_METHOD == EXHAUSTIVE_FAST) {
      // FAST corner detection
      // TODO: There is something wrong with fast9_detect destabilizing FPS. This problem is reduced with putting min_distance
      // to 0 (see defines), however a more permanent solution should be considered
      fast9_detect(&opticflow->prev_img_gray, opticflow->fast9_threshold, opticflow->fast9_min_distance,
                   opticflow->fast9_padding, opticflow->fast9_padding, &result->corner_cnt,
                   &opticflow->fast9_rsize,
                   &opticflow->fast9_ret_corners,
                   NULL);

    }
    else if (CORNER_METHOD == ACT_FAST) {
        // ACT-FAST corner detection:
        // TODO: all relevant things should be settings:
        float long_step = 10; // 5
        float short_step = 2; // 2
        int min_gradient = 10; // 10
        //printf("opticflow->fast9_threshold = %d, n_agents = %d, n_time_steps = %d\n", opticflow->fast9_threshold, n_agents, n_time_steps);
        act_fast(&opticflow->prev_img_gray, opticflow->fast9_threshold, &result->corner_cnt,
            &opticflow->fast9_ret_corners, n_agents, n_time_steps,
            long_step, short_step, min_gradient);
    }

    float post_corner = get_sys_time_float();
    //printf("Time spent on corner detection: %f\n", post_corner - pre_corner);

    // Adaptive threshold
    if (opticflow->fast9_adaptive) {

        // This works well for exhaustive FAST, but drives the threshold to the minimum for ACT-FAST:
      // Decrease and increase the threshold based on previous values
      if (result->corner_cnt < 40) { // TODO: Replace 40 with OPTICFLOW_MAX_TRACK_CORNERS / 2
        // make detections easier:
        if(opticflow->fast9_threshold > FAST9_LOW_THRESHOLD) {
            opticflow->fast9_threshold--;
        }

        if(CORNER_METHOD == ACT_FAST) {
            n_time_steps++;
            n_agents++;
        }

      } else if (result->corner_cnt > OPTICFLOW_MAX_TRACK_CORNERS * 2 && opticflow->fast9_threshold < FAST9_HIGH_THRESHOLD) {
        opticflow->fast9_threshold++;
        if(CORNER_METHOD == ACT_FAST && n_time_steps > 5 && n_agents > 10) {
            n_time_steps--;
            n_agents--;
        }
      }
    }
  }



#if OPTICFLOW_SHOW_CORNERS
  uint8_t color_points[4] = {127, 255, 127, 255};
  image_show_points_color(img, opticflow->fast9_ret_corners, result->corner_cnt, color_points);
#endif

  // Check if we found some corners to track
  if (result->corner_cnt < 1) {
    // Clear the result otherwise the previous values will be returned for this frame too
    memset(result, 0, sizeof(struct opticflow_result_t));
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    return;
  }

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************

  // Execute a Lucas Kanade optical flow
  result->tracked_cnt = result->corner_cnt;
  float pre_flow = get_sys_time_float();
  struct flow_t *vectors = opticFlowLK(&opticflow->img_gray, &opticflow->prev_img_gray, opticflow->fast9_ret_corners,
                                       &result->tracked_cnt,
                                       opticflow->window_size / 2, opticflow->subpixel_factor, opticflow->max_iterations,
                                       opticflow->threshold_vec, opticflow->max_track_corners, opticflow->pyramid_level);
  float post_flow = get_sys_time_float();
  // printf("Time spent on flow: %f\n", post_flow - pre_flow);
#if OPTICFLOW_SHOW_FLOW
  image_show_flow(img, vectors, result->tracked_cnt, opticflow->subpixel_factor);
#endif

  if(DOWNSELECT_VECTORS) {

    // Now we only take vectors into account when they are similar to the colors in the center of the view
    // TODO: take the color there were vectors are largest
    // get color histogram in the center.
    int UV_histogram[N_BINS_UV*N_BINS_UV];
    int histogram[N_BINS_UV*N_BINS_UV];
    int hist_distance;
    int cell_size = img->h / N_CELLS;
    int n_y_cells = img->h / cell_size;
    int n_x_cells = img->w / cell_size;
    int c, r;
    int c_center = n_x_cells / 2;
    int r_center = n_y_cells / 2;
    int x_min = c_center * cell_size;
    int x_max = (c_center+1) * cell_size;
    int y_min = r_center * cell_size;
    int y_max = (r_center+1) * cell_size;
    // printf("ROI UV histogram = %d, %d, %d, %d\n", x_min, x_max, y_min, y_max);
    // TODO: img may have changed by drawing on it!!!!!!! if OPTICFLOW_SHOW_FLOW is true for example.
    get_YUV_histogram(img, x_min, x_max, y_min, y_max, opticflow->n_samples_color_histogram, UV_histogram, N_BINS_UV, opticflow->color_min_UV, opticflow->color_max_UV);

    int cell_ind = 0;
    //int distances_histograms[n_x_cells*n_y_cells];
    int similarity_histograms[n_x_cells*n_y_cells];
    for(c = 0; c < n_x_cells; c++) {
        for(r = 0; r < n_y_cells; r++) {
            x_min = c * cell_size;
            x_max = (c+1) * cell_size;
            y_min = r * cell_size;
            y_max = (r+1) * cell_size;

            if(c == c_center && r == r_center) {
                // the center is what we sampled from, so it is always "in the game"
              hist_distance = 0;
            }
            else {
              get_YUV_histogram(img, x_min, x_max, y_min, y_max, opticflow->n_samples_color_histogram, histogram, N_BINS_UV, opticflow->color_min_UV, opticflow->color_max_UV);
              hist_distance = get_hist_distance(histogram, UV_histogram);
            }
            //distances_histograms[cell_ind] = hist_distance;
            similarity_histograms[cell_ind] = hist_distance <= opticflow->color_similarity_threshold;

            cell_ind++;
        }
    }

    if(OPTICFLOW_SHOW_INLIERS) {
        uint8_t color_rect[4] = {127, 255, 127, 255};
        uint8_t colborder = 10;
        uint8_t avg_color[4] = {0, 127, 255, 127};
      for(c = 0; c < n_x_cells; c++) {
            for(r = 0; r < n_y_cells; r++) {
                x_min = c * cell_size;
                x_max = (c+1) * cell_size;
                c_center = (x_max + x_min) / 2;
                y_min = r * cell_size;
                y_max = (r+1) * cell_size;
                r_center = (y_max + y_min) / 2;
                // draw the cell:
                image_draw_rectangle_color(img, x_min, x_max, y_min, y_max, color_rect);


                // show the center pixel's color:
                if(c_center % 2 == 1) c_center--;
                uint32_t buf_loc = c_center * 2 + r_center * img->w * 2;
                uint8_t *img_buf = (uint8_t *)img->buf;
                avg_color[0] = img_buf[buf_loc];
                avg_color[1] = img_buf[buf_loc+1];
                avg_color[2] = img_buf[buf_loc+2];
                avg_color[3] = img_buf[buf_loc+3];


               // get average color and draw a colored rectangle:
               // get_average_YUV(img, x_min, x_max, y_min, y_max, opticflow->n_samples_color_histogram, avg_color);

               image_draw_rectangle_color(img, c_center-colborder, c_center+colborder, r_center-colborder, r_center+colborder, avg_color);
               image_draw_rectangle_color(img, c_center-(colborder-1), c_center+(colborder-1), r_center-(colborder-1), r_center+(colborder-1), avg_color);
               image_draw_rectangle_color(img, c_center-(colborder-2), c_center+(colborder-2), r_center-(colborder-2), r_center+(colborder-2), avg_color);

            }
      }
    }
    // down-select the vectors:
    int x_cell, y_cell, index, n_inliers;
    int inliers[result->tracked_cnt];
    n_inliers = 0;
    for(int i = 0; i < result->tracked_cnt; i++) {
        // is the vector in a cell that is similar to the color model?
        x_cell = (vectors[i].pos.x / opticflow->subpixel_factor) / cell_size;
        y_cell = (vectors[i].pos.y / opticflow->subpixel_factor) / cell_size;
        index = x_cell * n_y_cells + y_cell;
        if(similarity_histograms[index]) {
          inliers[i] = 1;
          n_inliers++;
        }
        else {
            inliers[i] = 0;
        }
    }
    // replace vectors with only the inliers:
    //down_select_inlier_flow_vectors(inliers, vectors, result->tracked_cnt, n_inliers);
    //result->tracked_cnt = n_inliers;

#if OPTICFLOW_SHOW_INLIERS


    struct point_t loc;
    uint8_t color[4];
    color[0] = 255;
    color[1] = 255;
    color[2] = 255;
    color[3] = 255;
    int size_crosshair = 5;
    for(int i = 0; i < result->tracked_cnt; i++) {
        // cross-hair, only draw if far enough from the border:
        if(inliers[i]) {
            // green if inlier:
            color[0] = 0; color[2] = 0;
        }
        else {
            // pink if outlier:
            color[0] = 255; color[2] = 255;
        }
        loc.x = vectors[i].pos.x / opticflow->subpixel_factor;
        loc.y = vectors[i].pos.y / opticflow->subpixel_factor;
        image_draw_crosshair(img, &loc, color, size_crosshair);

        /*
        if(vectors[i].pos.x / opticflow->subpixel_factor >= size_crosshair && vectors[i].pos.x / opticflow->subpixel_factor < img->w - size_crosshair
            && vectors[i].pos.y / opticflow->subpixel_factor >= size_crosshair && vectors[i].pos.y / opticflow->subpixel_factor < img->h - size_crosshair) {
           // printf("Inlier!\n");
          from.x = vectors[i].pos.x / opticflow->subpixel_factor - size_crosshair;
          from.y = vectors[i].pos.y / opticflow->subpixel_factor;
          to.x = vectors[i].pos.x / opticflow->subpixel_factor + size_crosshair;
          to.y = vectors[i].pos.y / opticflow->subpixel_factor;
          image_draw_line_color(img, &from, &to, color);
          from.x = vectors[i].pos.x / opticflow->subpixel_factor;
          from.y = vectors[i].pos.y / opticflow->subpixel_factor -size_crosshair;
          to.x = vectors[i].pos.x / opticflow->subpixel_factor;
          to.y = vectors[i].pos.y / opticflow->subpixel_factor +size_crosshair;
          image_draw_line_color(img, &from, &to, color);
        }*/
    }
#endif
  }

  // Estimate size divergence:
  if (SIZE_DIV) {
    n_samples = 100;
    size_divergence = get_size_divergence(vectors, result->tracked_cnt, n_samples);
    result->div_size = size_divergence;
  } else {
    result->div_size = 0.0f;
  }
  if (LINEAR_FIT) {
    // Linear flow fit (normally derotation should be performed before):
    error_threshold = 10.0f;
    n_iterations_RANSAC = 20;
    n_samples_RANSAC = 5;
    success_fit = analyze_linear_flow_field(vectors, result->tracked_cnt, error_threshold, n_iterations_RANSAC,
                                            n_samples_RANSAC, img->w, img->h, &fit_info);

    if (!success_fit) {
      fit_info.divergence = 0.0f;
      fit_info.surface_roughness = 0.0f;
    }

    result->divergence = fit_info.divergence;
    result->surface_roughness = fit_info.surface_roughness;
  } else {
    result->divergence = 0.0f;
    result->surface_roughness = 0.0f;
  }


  // Get the median flow
  qsort(vectors, result->tracked_cnt, sizeof(struct flow_t), cmp_flow);
  if (result->tracked_cnt == 0) {
    // We got no flow
    result->flow_x = 0;
    result->flow_y = 0;
  } else if (result->tracked_cnt > 3) {
    // Take the average of the 3 median points
    result->flow_x = vectors[result->tracked_cnt / 2 - 1].flow_x;
    result->flow_y = vectors[result->tracked_cnt / 2 - 1].flow_y;
    result->flow_x += vectors[result->tracked_cnt / 2].flow_x;
    result->flow_y += vectors[result->tracked_cnt / 2].flow_y;
    result->flow_x += vectors[result->tracked_cnt / 2 + 1].flow_x;
    result->flow_y += vectors[result->tracked_cnt / 2 + 1].flow_y;
    result->flow_x /= 3;
    result->flow_y /= 3;
  } else {
    // Take the median point
    result->flow_x = vectors[result->tracked_cnt / 2].flow_x;
    result->flow_y = vectors[result->tracked_cnt / 2].flow_y;
  }


  // Flow Derotation
  float diff_flow_x = 0;
  float diff_flow_y = 0;

  /*// Flow Derotation TODO:
  float diff_flow_x = (cam_state->phi - opticflow->prev_phi) * img->w / OPTICFLOW_FOV_W;
  float diff_flow_y = (cam_state->theta - opticflow->prev_theta) * img->h / OPTICFLOW_FOV_H;*/

  if (opticflow->derotation && result->tracked_cnt > 5) {
    diff_flow_x = (cam_state->rates.p)  / result->fps * img->w /
                  OPTICFLOW_FOV_W;// * img->w / OPTICFLOW_FOV_W;
    diff_flow_y = (cam_state->rates.q) / result->fps * img->h /
                  OPTICFLOW_FOV_H;// * img->h / OPTICFLOW_FOV_H;
  }

  // *******
  // LOGGING
  // *******
  int16_t flow_threshold = 2;
  float min_time_interval_logging = 2.0;
  float pre_log_time = get_sys_time_float();
  if(LOG_DEROTATION) {
      // delta_phi,delta_theta,flow_x,flow_y
      fprintf(ofc_file_logger, "%f,%f,%f,%f,%f,%f,%f\n", (cam_state->rates.p)  / result->fps, (cam_state->rates.q)  / result->fps, (cam_state->rates.r)  / result->fps, ((float)result->flow_x)/opticflow->subpixel_factor, ((float)result->flow_y)/opticflow->subpixel_factor, diff_flow_x * opticflow->derotation_correction_factor_x, diff_flow_y * opticflow->derotation_correction_factor_y);
      //fprintf(ofc_file_logger, "%f,%f,%f,%f,%f\n", (cam_state->phi - opticflow->prev_phi)  / result->fps, (cam_state->theta - opticflow->prev_theta)  / result->fps, (cam_state->rates.r)  / result->fps, ((float)result->flow_x)/opticflow->subpixel_factor, ((float)result->flow_y)/opticflow->subpixel_factor);
  }
  else if(LOG_SSL){
      if(oscillating && abs(result->flow_x) > flow_threshold * opticflow->subpixel_factor ) {
          float curr_time = get_sys_time_float();
          // we should not log too much, since it takes quite some time.
          // moreover, the landing module should be active, so it should not have run too long ago
          if(curr_time - previous_log_time > min_time_interval_logging && curr_time - last_time_ofc_run < 0.5) {
              // don't make images when oscillating too little or too much:
              if(cov_div >= of_landing_ctrl.cov_set_point - 0.025 && cov_div <= of_landing_ctrl.cov_set_point + 0.025) {
                  log_SSL_information(vectors, result->tracked_cnt, opticflow, img);
                  previous_log_time = curr_time;
              }
          }
      }
  }
  float post_log_time = get_sys_time_float();
  printf("Logging time: %f seconds.\n", post_log_time - pre_log_time);


  result->flow_der_x = result->flow_x - diff_flow_x * opticflow->subpixel_factor *
                       opticflow->derotation_correction_factor_x;
  result->flow_der_y = result->flow_y - diff_flow_y * opticflow->subpixel_factor *
                       opticflow->derotation_correction_factor_y;
  opticflow->prev_rates = cam_state->rates;

  // Velocity calculation
  // Right now this formula is under assumption that the flow only exist in the center axis of the camera.
  // TODO Calculate the velocity more sophisticated, taking into account the drone's angle and the slope of the ground plane.
  float vel_x = result->flow_der_x * result->fps * cam_state->agl / opticflow->subpixel_factor  / OPTICFLOW_FX;
  float vel_y = result->flow_der_y * result->fps * cam_state->agl / opticflow->subpixel_factor  / OPTICFLOW_FY;

  //Apply a  median filter to the velocity if wanted
  if (opticflow->median_filter == true) {
    result->vel_x = (float)update_median_filter_i(&vel_x_filt, (int32_t)(vel_x * 1000)) / 1000;
    result->vel_y = (float)update_median_filter_i(&vel_y_filt, (int32_t)(vel_y * 1000)) / 1000;
  } else {
    result->vel_x = vel_x;
    result->vel_y = vel_y;
  }
  // Velocity calculation: uncomment if focal length of the camera is not known or incorrect.
  //  result->vel_x =  - result->flow_der_x * result->fps * cam_state->agl / opticflow->subpixel_factor * OPTICFLOW_FOV_W / img->w
  //  result->vel_y =  result->flow_der_y * result->fps * cam_state->agl / opticflow->subpixel_factor * OPTICFLOW_FOV_H / img->h


  // Correct also the flow for the FPS:
  result->flow_x *= result->fps;
  result->flow_y *= result->fps;
  result->flow_der_x *= result->fps;
  result->flow_der_y *= result->fps;

  // Determine quality of noise measurement for state filter
  //TODO develop a noise model based on groundtruth

  float noise_measurement_temp = (1 - ((float)result->tracked_cnt / ((float)opticflow->max_track_corners * 1.25)));
  result->noise_measurement = noise_measurement_temp;

  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************

  if (opticflow->feature_management) {
    result->corner_cnt = result->tracked_cnt;
    //get the new positions of the corners and the "residual" subpixel positions
    for (uint16_t i = 0; i < result->tracked_cnt; i++) {
      opticflow->fast9_ret_corners[i].x = (uint32_t)((vectors[i].pos.x + (float)vectors[i].flow_x) / opticflow->subpixel_factor);
      opticflow->fast9_ret_corners[i].y = (uint32_t)((vectors[i].pos.y + (float)vectors[i].flow_y) / opticflow->subpixel_factor);
      opticflow->fast9_ret_corners[i].x_sub = (uint16_t)((vectors[i].pos.x + vectors[i].flow_x) % opticflow->subpixel_factor);
      opticflow->fast9_ret_corners[i].y_sub = (uint16_t)((vectors[i].pos.y + vectors[i].flow_y) % opticflow->subpixel_factor);
      opticflow->fast9_ret_corners[i].count = vectors[i].pos.count;
    }
  }
  free(vectors);
  image_switch(&opticflow->img_gray, &opticflow->prev_img_gray);
}

/**
 * Run the optical flow with EDGEFLOW on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
void calc_edgeflow_tot(struct opticflow_t *opticflow, struct opticflow_state_t *cam_state, struct image_t *img,
                       struct opticflow_result_t *result)
{
  // Define Static Variables
  static struct edge_hist_t edge_hist[MAX_HORIZON];
  static uint8_t current_frame_nr = 0;
  struct edge_flow_t edgeflow;
  static uint8_t previous_frame_offset[2] = {1, 1};

  // Define Normal variables
  struct edgeflow_displacement_t displacement;
  displacement.x = malloc(sizeof(int32_t) * img->w);
  displacement.y = malloc(sizeof(int32_t) * img->h);

  // If the methods just switched to this one, reintialize the
  // array of edge_hist structure.
  if (opticflow->just_switched_method == 1) {
    int i;
    for (i = 0; i < MAX_HORIZON; i++) {
      edge_hist[i].x = malloc(sizeof(int32_t) * img->w);
      edge_hist[i].y = malloc(sizeof(int32_t) * img->h);
      FLOAT_RATES_ZERO(edge_hist[i].rates);
    }
  }

  uint16_t disp_range;
  if (opticflow->search_distance < DISP_RANGE_MAX) {
    disp_range = opticflow->search_distance;
  } else {
    disp_range = DISP_RANGE_MAX;
  }

  uint16_t window_size;

  if (opticflow->window_size < MAX_WINDOW_SIZE) {
    window_size = opticflow->window_size;
  } else {
    window_size = MAX_WINDOW_SIZE;
  }

  uint16_t RES = opticflow->resolution_factor;

  //......................Calculating EdgeFlow..................... //

  // Calculate current frame's edge histogram
  int32_t *edge_hist_x = edge_hist[current_frame_nr].x;
  int32_t *edge_hist_y = edge_hist[current_frame_nr].y;
  calculate_edge_histogram(img, edge_hist_x, 'x', 0);
  calculate_edge_histogram(img, edge_hist_y, 'y', 0);


  // Copy frame time and angles of image to calculated edge histogram
  edge_hist[current_frame_nr].frame_time = img->ts;
  edge_hist[current_frame_nr].rates = cam_state->rates;

  // Calculate which previous edge_hist to compare with the current
  uint8_t previous_frame_nr[2];
  calc_previous_frame_nr(result, opticflow, current_frame_nr, previous_frame_offset, previous_frame_nr);

  //Select edge histogram from the previous frame nr
  int32_t *prev_edge_histogram_x = edge_hist[previous_frame_nr[0]].x;
  int32_t *prev_edge_histogram_y = edge_hist[previous_frame_nr[1]].y;

  //Calculate the corresponding derotation of the two frames
  int16_t der_shift_x = 0;
  int16_t der_shift_y = 0;

  if (opticflow->derotation) {
      // TODO: make this nice and general, so that p / q/ r are adapted to front / bottom cam!!!
    der_shift_x = (int16_t)(edge_hist[current_frame_nr].rates.q  /
                            result->fps *
                            (float)img->w / (OPTICFLOW_FOV_W) * opticflow->derotation_correction_factor_x);
    der_shift_y = (int16_t)(edge_hist[current_frame_nr].rates.r /
                            result->fps *
                            (float)img->h / (OPTICFLOW_FOV_H) * opticflow->derotation_correction_factor_y);
  }

  // Estimate pixel wise displacement of the edge histograms for x and y direction
  calculate_edge_displacement(edge_hist_x, prev_edge_histogram_x,
                              displacement.x, img->w,
                              window_size, disp_range,  der_shift_x);
  calculate_edge_displacement(edge_hist_y, prev_edge_histogram_y,
                              displacement.y, img->h,
                              window_size, disp_range, der_shift_y);

  // Fit a line on the pixel displacement to estimate
  // the global pixel flow and divergence (RES is resolution)
  line_fit(displacement.x, &edgeflow.div_x,
           &edgeflow.flow_x, img->w,
           window_size + disp_range, RES);
  line_fit(displacement.y, &edgeflow.div_y,
           &edgeflow.flow_y, img->h,
           window_size + disp_range, RES);

  /* Save Resulting flow in results
   * Warning: The flow detected here is different in sign
   * and size, therefore this will be divided with
   * the same subpixel factor and -1 to make it on par with
   * the LK algorithm of t opticalflow_calculator.c
   * */
  edgeflow.flow_x = -1 * edgeflow.flow_x;
  edgeflow.flow_y = -1 * edgeflow.flow_y;

  edgeflow.flow_x = (int16_t)edgeflow.flow_x / previous_frame_offset[0];
  edgeflow.flow_y = (int16_t)edgeflow.flow_y / previous_frame_offset[1];

  result->flow_x = (int16_t)edgeflow.flow_x / RES;
  result->flow_y = (int16_t)edgeflow.flow_y / RES;

  //Fill up the results optic flow to be on par with LK_fast9
  result->flow_der_x =  result->flow_x;
  result->flow_der_y =  result->flow_y;
  result->corner_cnt = getAmountPeaks(edge_hist_x, 500 , img->w);
  result->tracked_cnt = getAmountPeaks(edge_hist_x, 500 , img->w);
  result->divergence = (float)edgeflow.div_x / RES;
  result->div_size = 0.0f;
  result->noise_measurement = 0.0f;
  result->surface_roughness = 0.0f;

  //......................Calculating VELOCITY ..................... //

  /*Estimate fps per direction
   * This is the fps with adaptive horizon for subpixel flow, which is not similar
   * to the loop speed of the algorithm. The faster the quadcopter flies
   * the higher it becomes
  */
  float fps_x = 0;
  float fps_y = 0;
  float time_diff_x = (float)(timeval_diff(&edge_hist[previous_frame_nr[0]].frame_time, &img->ts)) / 1000.;
  float time_diff_y = (float)(timeval_diff(&edge_hist[previous_frame_nr[1]].frame_time, &img->ts)) / 1000.;
  fps_x = 1 / (time_diff_x);
  fps_y = 1 / (time_diff_y);

  result->fps = fps_x;

  // Calculate velocity
  float vel_x = edgeflow.flow_x * fps_x * cam_state->agl * OPTICFLOW_FOV_W / (img->w * RES);
  float vel_y = edgeflow.flow_y * fps_y * cam_state->agl * OPTICFLOW_FOV_H / (img->h * RES);

  //Apply a  median filter to the velocity if wanted
  if (opticflow->median_filter == true) {
    result->vel_x = (float)update_median_filter_i(&vel_x_filt, (int32_t)(vel_x * 1000)) / 1000;
    result->vel_y = (float)update_median_filter_i(&vel_y_filt, (int32_t)(vel_y * 1000)) / 1000;
  } else {
    result->vel_x = vel_x;
    result->vel_y = vel_y;
  }

  result->noise_measurement = 0.2;



#if OPTICFLOW_SHOW_FLOW
  draw_edgeflow_img(img, edgeflow, prev_edge_histogram_x, edge_hist_x);
#endif
  // Increment and wrap current time frame
  current_frame_nr = (current_frame_nr + 1) % MAX_HORIZON;

  // Free malloc'd variables
  free(displacement.x);
  free(displacement.y);
}


/**
 * Run the optical flow on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
void opticflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *cam_state, struct image_t *img,
                          struct opticflow_result_t *result)
{

  // A switch counter that checks in the loop if the current method is similar,
  // to the previous (for reinitializing structs)
  static int8_t switch_counter = -1;
  if (switch_counter != opticflow->method) {
    opticflow->just_switched_method = true;
    switch_counter = opticflow->method;
    // Clear the static result
    memset(result, 0, sizeof(struct opticflow_result_t));
  } else {
    opticflow->just_switched_method = false;
  }

  // Switch between methods (0 = fast9/lukas-kanade, 1 = EdgeFlow)
  if (opticflow->method == 0) {
    calc_fast9_lukas_kanade(opticflow, cam_state, img, result);
  } else if (opticflow->method == 1) {
    calc_edgeflow_tot(opticflow, cam_state, img, result);
  }

  /* Rotate velocities from camera frame coordinates to body coordinates for control
  * IMPORTANT!!! This frame to body orientation should be the case for the Parrot
  * ARdrone and Bebop, however this can be different for other quadcopters
  * ALWAYS double check!
  */
  result->vel_body_x = result->vel_y;
  result->vel_body_y = - result->vel_x;

  // KALMAN filter on velocity
  float measurement_noise[2] = {result->noise_measurement, 1.0f};
  static bool reinitialize_kalman = true;

  static uint8_t wait_filter_counter =
    0; // When starting up the opticalflow module, or switching between methods, wait for a bit to prevent bias


  if (opticflow->kalman_filter == true) {
    if (opticflow->just_switched_method == true) {
      wait_filter_counter = 0;
      reinitialize_kalman = true;
    }

    if (wait_filter_counter > 50) {

      // Get accelerometer values rotated to body axis
      // TODO: use acceleration from the state ?
      struct FloatVect3 accel_imu_f;
      ACCELS_FLOAT_OF_BFP(accel_imu_f, cam_state->accel_imu_meas);
      struct FloatVect3 accel_meas_body;
      float_quat_vmult(&accel_meas_body, &cam_state->imu_to_body_quat, &accel_imu_f);

      float acceleration_measurement[2];
      acceleration_measurement[0] = accel_meas_body.x;
      acceleration_measurement[1] = accel_meas_body.y;

      kalman_filter_opticflow_velocity(&result->vel_body_x, &result->vel_body_y, acceleration_measurement, result->fps,
                                       measurement_noise, opticflow->kalman_filter_process_noise, reinitialize_kalman);
      if (reinitialize_kalman) {
        reinitialize_kalman = false;
      }

    } else {
      wait_filter_counter++;
    }
  } else {
    reinitialize_kalman = true;
  }

}

/**
 * Filter the velocity with a simple linear kalman filter, together with the accelerometers
 * @param[in] *velocity_x  Velocity in x direction of body fixed coordinates
 * @param[in] *velocity_y  Belocity in y direction of body fixed coordinates
 * @param[in] *acceleration_measurement  Measurements of the accelerometers
 * @param[in] fps  Frames per second
 * @param[in] *measurement_noise  Expected variance of the noise of the measurements
 * @param[in] *measurement_noise  Expected variance of the noise of the model prediction
 * @param[in] reinitialize_kalman  Boolean to reinitialize the kalman filter
 */
void kalman_filter_opticflow_velocity(float *velocity_x, float *velocity_y, float *acceleration_measurement, float fps,
                                      float *measurement_noise, float kalman_process_noise, bool reinitialize_kalman)
{
  // Initialize variables
  static float covariance_x[4], covariance_y[4], state_estimate_x[2], state_estimate_y[2];
  float measurements_x[2], measurements_y[2];

  if (reinitialize_kalman) {
    state_estimate_x[0] = 0.0f;
    state_estimate_x[1] = 0.0f;
    covariance_x[0] = 1.0f;
    covariance_x[1] = 1.0f;
    covariance_x[2] = 1.0f;
    covariance_x[3] = 1.0f;

    state_estimate_y[0] = 0.0f;
    state_estimate_y[1] = 0.0f;
    covariance_y[0] = 1.0f;
    covariance_y[1] = 1.0f;
    covariance_y[2] = 1.0f;
    covariance_y[3] = 1.0f;
  }

  /*Model for velocity estimation
   * state = [ velocity; acceleration]
   * Velocity_prediction = last_velocity_estimate + acceleration * dt
   * Acceleration_prediction = last_acceleration
   * model = Jacobian([vel_prediction; accel_prediction],state)
   *       = [1 dt ; 0 1];
   * */
  float model[4] =  {1.0f, 1.0f / fps , 0.0f , 1.0f};
  float process_noise[2] = {kalman_process_noise, kalman_process_noise};

  // Measurements from velocity_x of optical flow and acceleration directly from scaled accelerometers
  measurements_x[0] = *velocity_x;
  measurements_x[1] = acceleration_measurement[0];

  measurements_y[0] = *velocity_y;
  measurements_y[1] = acceleration_measurement[1];

  // 2D linear kalman filter
  kalman_filter_linear_2D_float(model, measurements_x, covariance_x, state_estimate_x, process_noise, measurement_noise);
  kalman_filter_linear_2D_float(model, measurements_y, covariance_y, state_estimate_y, process_noise, measurement_noise);

  *velocity_x = state_estimate_x[0];
  *velocity_y = state_estimate_y[0];
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
  msec = (finishtime->tv_sec - starttime->tv_sec) * 1000;
  msec += (finishtime->tv_usec - starttime->tv_usec) / 1000;
  return msec;
}

/**
 * Compare two flow vectors based on flow distance
 * Used for sorting.
 * @param[in] *a The first flow vector (should be vect flow_t)
 * @param[in] *b The second flow vector (should be vect flow_t)
 * @return Negative if b has more flow than a, 0 if the same and positive if a has more flow than b
 */
static int cmp_flow(const void *a, const void *b)
{
  const struct flow_t *a_p = (const struct flow_t *)a;
  const struct flow_t *b_p = (const struct flow_t *)b;
  return (a_p->flow_x * a_p->flow_x + a_p->flow_y * a_p->flow_y) - (b_p->flow_x * b_p->flow_x + b_p->flow_y *
         b_p->flow_y);
}

/**
 * Compare the rows of an integer (uint16_t) 2D array based on the first column.
 * Used for sorting.
 * @param[in] *a The first row (should be *uint16_t)
 * @param[in] *b The second flow vector (should be *uint16_t)
 * @return Negative if a[0] < b[0],0 if a[0] == b[0] and positive if a[0] > b[0]
 */
static int cmp_array(const void *a, const void *b)
{
  const uint16_t *pa = *(const uint16_t **)a;
  const uint16_t *pb = *(const uint16_t **)b;
  return pa[0] - pb[0];
}

void get_average_YUV(struct image_t *img, int x_min, int x_max, int y_min, int y_max, int n_samples, uint8_t *color) {
  // get average YUV values in a region, in the color format [U Y V Y]
  uint8_t *source = img->buf;
   int x, y;
   int x_range = x_max - x_min;
   int y_range = y_max - y_min;
   uint16_t index;
   uint16_t pixel_step = 2;
   uint16_t row_stride = img->w * pixel_step;
   uint8_t U,V,Y;
   uint32_t avg_U = 0;
   uint32_t avg_V = 0;
   uint32_t avg_Y = 0;
   for(int s = 0; s < n_samples; s++) {
       // generate random coordinates:
       x = rand() % x_range + x_min;
       // only accept even x due to UYVY format
       x = (x % 2 == 0) ? x : x-1;
       y = rand() % y_range + y_min;

       // determine the index of the pixel in the UYVY image and get the values:
       index = y * row_stride + x * pixel_step;
       U = source[index];
       Y = source[index+1]; // source[index+3]
       V = source[index+2];

       avg_U += U;
       avg_V += V;
       avg_Y += Y;
  }
  // average Y, U, V:
  avg_U /= n_samples;
  avg_V /= n_samples;
  avg_Y /= n_samples;
  // set the colors:
  color[0] = (uint8_t) avg_U;
  color[1] = (uint8_t) avg_Y;
  color[2] = (uint8_t) avg_V;
  color[3] = color[1];
}

void get_YUV_histogram(struct image_t *img, int x_min, int x_max, int y_min, int y_max, int n_samples, int* histogram, int n_bins_UV, int min_col, int max_col) {
  // get a UV histogram for now (Y to be added later):
  // sample pixels at random from the given region of interest

  // printf("\n\nCELL\n\n");

  // empty the histogram
  for(int i = 0; i < n_bins_UV * n_bins_UV; i++) {
      histogram[i] = 0;
  }
  uint8_t *source = img->buf;
  int x, y, bin_U, bin_V;
  int x_range = x_max - x_min;
  int y_range = y_max - y_min;
  uint16_t index;
  uint16_t pixel_step = 2;
  uint16_t row_stride = img->w * pixel_step;
  uint8_t U,V,Y;
  int col_range = max_col - min_col;
  int bin_size;
  if(col_range == 255) {
      bin_size = col_range / n_bins_UV;
  }
  else {
      bin_size = col_range / (n_bins_UV - 2);
  }
  // printf("Min col = %d, max col = %d, range = %d, bin_size = %d\n", min_col, max_col, col_range, bin_size);
  for(int s = 0; s < n_samples; s++) {
      // generate random coordinates:
      x = rand() % x_range + x_min;
      // only accept even x due to UYVY format
      x = (x % 2 == 0) ? x : x-1;
      y = rand() % y_range + y_min;

      // determine the index of the pixel in the UYVY image and get the values:
      index = y * row_stride + x * pixel_step;
      U = source[index];
      Y = source[index+1]; // source[index+3]
      V = source[index+2];
      // printf("(%d, %d, %d) ", U, V, Y);

      // determine the corresponding bins and place in the array:
      /*if(Y >= 25) {*/
          // if bright enough, bin the color:
      if(min_col == 0 && max_col == 255) {
          bin_U = U / bin_size;
          bin_V = V / bin_size;
      }
      else {
          if(U <= min_col) {
              bin_U = 0;
          }
          else if(U >= max_col) {
              bin_U = n_bins_UV - 1;
          }
          else {
              bin_U = (U - min_col) / bin_size;
          }
          if(V <= min_col) {
              bin_V = 0;
          }
          else if(V >= max_col) {
              bin_V = n_bins_UV - 1;
          }
          else {
              bin_V = (V - min_col) / bin_size;
          }

      }
          /*
      }
      else {
          // if too dark, put it as grey:
          bin_U = n_bins_UV / 2;
          bin_V = n_bins_UV / 2;
      }*/

      // printf("Index = %d\n", bin_U * n_bins_UV + bin_V);
      histogram[bin_U * n_bins_UV + bin_V]++;
  }

  /*for(int i = 0; i < N_BINS_UV * N_BINS_UV; i++) {
    printf("%d ", histogram[i]);
  }
  printf("\n");*/
}

int get_hist_distance(int* hist, int* model_hist)
{

  printf("\n\nHisto: ");
  for(int i = 0; i < N_BINS_UV * N_BINS_UV; i++) {
    printf("%d ", hist[i]);
  }
  printf("\n");
  printf("Model: ");
    for(int i = 0; i < N_BINS_UV * N_BINS_UV; i++) {
      printf("%d ", model_hist[i]);
    }
    printf("\n");


        int dist = 0;
        int add_dist;
        for (int i = 0; i < N_BINS_UV * N_BINS_UV; i++)
        {
          add_dist = abs(hist[i] - model_hist[i]);
          //printf("add_dist = %d ", add_dist);
          dist += add_dist;
        }
        //printf("\nTotal dist = %d\n", dist);
        return dist;
}

/*
void down_select_inlier_flow_vectors(int* inliers, struct flow_t* vectors, int n_vectors, int n_inliers) {
  // only retain vectors when they are inliers:
  struct flow_t *new_vectors = malloc(sizeof(struct flow_t) * n_inliers);
  int i_inlier = 0;
  for(int i = 0; i < n_vectors; i++) {
      if(inliers[i]) {
        new_vectors[i_inlier].flow_x = vectors[i].flow_x;
        new_vectors[i_inlier].flow_y = vectors[i].flow_y;
        // TODO: check if this does not give problems when freeing:
        new_vectors[i_inlier].pos = vectors[i].pos;
        i_inlier++;
      }
  }
  // free the previously assigned memory in vectors
  free(vectors);
  // set the vectors reference to the new vector array, only filled with inliers:
  vectors = new_vectors;
}
*/

void undistort_flow(struct flow_t *vectors, uint16_t n_vectors, struct opticflow_t *opticflow, struct flow_t *undistorted_vectors) {

  // parameters should be settings:
  // cameraParams_2coeff.FocalLength =    332.5259  341.2480
  // cameraParams_2coeff.PrincipalPoint =   179.5224  348.8104
  float focal_length[2] = {332.5259, 341.2480};
  float principal_point[2] = {179.5224, 348.8104};
  float params[7] = {-0.002197, -0.027555, 0.338736, -0.102699, 0.015190, 0.060531, 0.058638};
  float r2, r4, r6;
  float centered_x[2], centered_y[2], undistorted_x[2], undistorted_y[2], radii[2], undistorted_flow[2];
  for(int v = 0; v < n_vectors; v++) {
      // distorted radius = norm of centered point
      // A = [ones(N,1), distorted_radii, distorted_radii.^2, distorted_radii.^4, distorted_radii.^6, centered_points(:,1), centered_points(:,2)];
      // Par = -0.002197 -0.027555 0.338736 -0.102699 0.015190 0.060531 0.058638
      // undistorted = A * par

      // make sure that both the starting and end point of the flow vector are represented with 0,0 the center of the image
      centered_x[0] = ((float) vectors[v].pos.x / opticflow->subpixel_factor - principal_point[0]) / focal_length[0];
      centered_y[0] = ((float) vectors[v].pos.y / opticflow->subpixel_factor - principal_point[1]) / focal_length[1];
      radii[0] = sqrtf(centered_x[0]*centered_x[0] + centered_y[0]*centered_y[0]);
      centered_x[1] = ((float) ((int16_t) vectors[v].pos.x + vectors[v].flow_x) / opticflow->subpixel_factor - principal_point[0]) / focal_length[0];
      centered_y[1] = ((float) ((int16_t) vectors[v].pos.y + vectors[v].flow_y) / opticflow->subpixel_factor - principal_point[1]) / focal_length[1];
      radii[1] = sqrtf(centered_x[1]*centered_x[1] + centered_y[1]*centered_y[1]);

      // get the undistorted coordinates:
      r2 = radii[0]*radii[0];
      r4 = r2*r2;
      r6 = r4*r2;
      undistorted_x[0] = centered_x[0] + params[0] + params[1]*radii[0] + params[2]*r2 + params[3]*r4 + params[4]*r6 + params[5]*centered_x[0] + params[6]*centered_y[0];
      undistorted_y[0] = centered_y[0] + params[0] + params[1]*radii[0] + params[2]*r2 + params[3]*r4 + params[4]*r6 + params[5]*centered_x[0] + params[6]*centered_y[0];
      r2 = radii[1]*radii[1];
      r4 = r2*r2;
      r6 = r4*r2;
      undistorted_x[1] = centered_x[1] + params[0] + params[1]*radii[1] + params[2]*r2 + params[3]*r4 + params[4]*r6 + params[5]*centered_x[1] + params[6]*centered_y[1];
      undistorted_y[1] = centered_y[1] + params[0] + params[1]*radii[1] + params[2]*r2 + params[3]*r4 + params[4]*r6 + params[5]*centered_x[1] + params[6]*centered_y[1];
      undistorted_flow[0] = undistorted_x[1] - undistorted_x[0];
      undistorted_flow[1] = undistorted_y[1] - undistorted_y[0];

      // put them in the output flow vector:
      undistorted_vectors[v].pos.x = (uint32_t) ((undistorted_x[0] * focal_length[0] + principal_point[0]) * opticflow->subpixel_factor);
      undistorted_vectors[v].pos.y = (uint32_t) ((undistorted_y[0] * focal_length[1] + principal_point[1]) * opticflow->subpixel_factor);
      undistorted_vectors[v].flow_x = (int16_t) (undistorted_flow[0] * focal_length[0] * opticflow->subpixel_factor);
      undistorted_vectors[v].flow_y = (int16_t) (undistorted_flow[1] * focal_length[1] * opticflow->subpixel_factor);
  }
}

void log_SSL_information(struct flow_t *vectors, uint16_t n_vectors, struct opticflow_t *opticflow, struct image_t *img) {
  // write the vectors to a file (always logging OPTICFLOW_MAX_TRACK_CORNERS to keep number of columns equal over time), together with the gains pstate, pused

  printf("Logging!!!!\n");
  // store the current color image:
  video_capture_save(img);
  // store the current gray-scale image:
  video_capture_save(&opticflow->prev_img_gray);

  //store the gain and vectors:
  fprintf(ofc_file_logger, "%f,%f,%d", cov_div, pused, n_vectors);
  for(int i = 0; i < OPTICFLOW_MAX_TRACK_CORNERS; i++) {
      if(i < n_vectors) {
          fprintf(ofc_file_logger, ",%f,%f,%f,%f", (float)vectors[i].pos.x, (float)vectors[i].pos.y, (float)vectors[i].flow_x, (float)vectors[i].flow_y);
      }
      else {
          // to ensure the same number of columns when there are fewer flow vectors:
          fprintf(ofc_file_logger, ",0.0,0.0,0.0,0.0");
      }
  }
  fprintf(ofc_file_logger, "\n");
}

