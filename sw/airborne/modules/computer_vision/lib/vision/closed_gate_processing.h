
/*
 * Copyright (C) 2016
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
 * @file modules/computer_vision/lib/vision/closed_gate_processing.h
 */

#include <stdint.h>
#include "modules/computer_vision/cv.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"

/* Gate structure */
struct gate_img {
  int x;             ///< The image x coordinate of the gate center
  int y;             ///< The image y coordinate of the gate center
  int x_corners[4];///< Array of corner x coordinates
  int y_corners[4];///< Array of corner y coordinates
  int sz;            ///< Half the image size of the gate 
  float gate_q;      ///< gate quality
  int n_sides;       ///< How many sides are orange (to prevent detecting a small gate in the corner of a big one partially out of view).
  float sz_left;     ///< Half the image size of the left side
  float sz_right;    ///< Half the image size of the right side
};

int check_color(struct image_t *im, int x, int y);
void snake_up_and_down(struct image_t *im, int x, int y, int* y_low, int* y_high);
void snake_left_and_right(struct image_t *im, int x, int y, int* x_low, int* x_high);
void draw_gate(struct image_t *im, struct gate_img gate);
void draw_gate_polygon(struct image_t *im, int *x_points, int *y_points, uint8_t* color);
void draw_gate_color(struct image_t *im, struct gate_img gate, uint8_t* color);
void check_gate(struct image_t *im, struct gate_img gate, float* quality, int* sides);
void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int* n_points, int* n_colored_points);
void check_gate_free(struct image_t *im, struct gate_img gate, float *quality, int *n_sides);

void vec_from_point(float point_x, float point_y, int f, struct FloatVect3 *vec);
void vec_from_point_2(float point_x, float point_y, int f, struct FloatVect3 *vec);
void vec_from_point_ned(float point_x, float point_y, int f, struct FloatVect3 *vec);
void undistort_fisheye_point(int point_x, int point_y, float *undistorted_x, float *undistorted_y, int f, float k, float x_img_center, float y_img_center);
void back_proj_points(struct FloatVect3 *gate_point, struct FloatVect3 *cam_pos, struct FloatMat33 *R_mat, float *x_res, float *y_res);
float euclidean_distance(float x_i, float x_bp, float y_i, float y_bp);
int find_minimum(float *error);

void get_hist_sub_sampling(struct image_t *im, int* raw_hist, int step_y);
void get_cumulative_hist(int* smart_histogram, float* smart_cumulative_histogram, int n_elements);
int get_coord_from_cumulative(int* smart_cumulative_histogram, int n_elements);
void smooth_hist(int *smooth, int *raw_hist, int window);
int find_max_hist(int *hist);
int find_hist_peeks(float *hist,int *peeks);
int find_hist_peeks_flat(int *hist,int *peeks);
void print_hist(struct image_t *img,int *hist);
void print_sides(struct image_t *im, int side_1, int side_2);
float detect_gate_sides(int *hist_raw, int *side_1, int *side_2, int* n_high_peaks);

void print_matrix(struct FloatMat33 mat);
void print_vector(struct FloatVect3 vec);
void draw_cross(struct image_t *im,int x, int y, uint8_t* color);

void check_color_center(struct image_t *im, uint8_t *y_c, uint8_t *cb_c, uint8_t *cr_c);
uint16_t image_yuv422_set_color(struct image_t *input, struct image_t *output, int x, int y);

int closed_gate_processing(struct image_t *img);

int color_count;

extern uint8_t green_color[4];
extern uint8_t blue_color[4];

extern float measured_x_gate;
extern float measured_y_gate;
extern float measured_z_gate;
extern float delta_z_gate;

extern float x_dist;

extern float current_x_gate;
extern float current_y_gate;
extern float current_z_gate;
extern char fitness;
extern int gate_detected;
extern int init_pos_filter;

extern float p3p_result_x;
extern float p3p_result_y;
extern float p3p_result_z;
extern float p3p_result_phi;
extern float p3p_result_theta;
extern float p3p_result_psi;
extern float snake_res_x;
extern float snake_res_y;
extern float snake_res_z;

//logging corner points in image plane
extern float gate_img_point_x_1;
extern float gate_img_point_y_1;
extern float gate_img_point_x_2;
extern float gate_img_point_y_2;
extern float gate_img_point_x_3;
extern float gate_img_point_y_3;
extern float gate_img_point_x_4;
extern float gate_img_point_y_4;

//hist final approach
extern float x_pos_hist;
extern float y_pos_hist;

//maybe shouldnt be extern?
extern float gate_dist_x;//distance from filter init point to gate 
extern float gate_size_m;//size of gate edges in meters
extern float gate_center_height;//height of gate in meters ned wrt ground
