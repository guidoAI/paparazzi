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
 * @file modules/computer_vision/lib/vision/open_gate_detection.c
 */

// Own header
#include "modules/computer_vision/lib/vision/open_gate_processing.h"
//#include "modules/computer_vision/lib/vision/closed_gate_processing.h"//for histogram stuf
#include "modules/computer_vision/snake_gate_detection.h"
#include <stdio.h>
#include "modules/computer_vision/lib/vision/image.h"
#include <stdlib.h>
#include "subsystems/datalink/telemetry.h"
#include "subsystems/imu.h"
//#include "modules/computer_vision/lib/vision/gate_detection.h"
#include "modules/computer_vision/lib/vision/gate_detection_free.h"
#include "modules/computer_vision/lib/vision/gate_corner_refine.h"
#include "state.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/lib/vision/qr_code_recognition.h"
#include "modules/computer_vision/lib/vision/calc_p3p.h"
#include "modules/first_stretch/first_stretch.h"

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"

#include "filters/low_pass_filter.h"
#include <inttypes.h>


#define PI 3.1415926

//initial position and speed safety margins

#define AHRS_PROPAGATE_FREQUENCY 512

#define HFF_LOWPASS_CUTOFF_FREQUENCY 14

// Gate detection settings:
int min_pixel_size_o = 40;//20;//30;//100;
int min_pixel_size_hor = 20;
float min_gate_quality_o = 0.15;//0.2;


// Result
int color_count = 0;
#define MAX_GATES 50
// struct gate_img gates[MAX_GATES];
// struct gate_img best_gate;
// struct gate_img temp_check_gate;
struct image_t img_result;


// my modification
struct opengate_img opengates[MAX_GATES];
int n_op_samples = 3000;//number for open gate samples.
float ox_dist = 0;
float oy_dist = 0;
float oz_dist = 0;
//struct FloatVect3 ogate_vectors[5];
//struct FloatVect3 ogate_points[5];
float ogate_dist_x = 3.50;
// my modification

//camera parameters
#define radians_per_pix_w 0.006666667//2.1 rad(60deg)/SNAKE_GATE_HEIGHT
#define radians_per_pix_h 0.0065625 //1.05rad / SNAKE_GATE_WIDTH

//Debug messages

//vectors for p3p etc
struct FloatVect3 vec_point_1, vec_point_2, vec_point_3, vec_point_4, vec_temp1, vec_temp2, vec_temp3, vec_temp4,
vec_ver_1, vec_ver_2, vec_ver_3, vec_ver_4;

struct FloatVect3 gate_vectors[4];
struct FloatVect3 gate_points[4];
struct FloatVect3 p3p_pos_solution;

float local_psi_o = 0;

int *array_o;
//return indexes
int cmp_i_o(const void *a, const void *b){
    int ia = *(int *)a;
    int ib = *(int *)b;
    return array_o[ia] < array_o[ib] ? -1 : array_o[ia] > array_o[ib];
}

float detect_gate_side_o(int *hist_raw, int *side){
  int hist_peeks[SNAKE_GATE_HEIGHT];//max nr of peeks, for sure
  int hist_smooth[SNAKE_GATE_HEIGHT];
  smooth_hist(hist_smooth, hist_raw, 4);//was 10
  int peek_count = find_hist_peeks_flat(hist_smooth,hist_peeks);
  int max_peek = find_max_hist(hist_raw);
  int size = sizeof(hist_peeks)/sizeof(*hist_peeks);
  int index[size];
  for(int i=0;i<size;i++){
        index[i] = i;
    }
  array_o = hist_peeks;
  qsort(index, size, sizeof(*index), cmp_i_o);
  //printf("p1:%d p2:%d p3:%d p4:%d p5:%d p6:%d\n",hist_raw[index[0]],hist_raw[index[1]],hist_raw[index[2]],hist_raw[index[3]],hist_raw[index[4]],hist_raw[index[5]]);
  // printf("size:%d\n",size);
  //   printf("\n\ndata\tindex\n");
  //     for(int i=0;i<size;i++){
  //         printf("%d\t%d,i:%d\n", hist_peeks[index[i]], index[i],i);
  //     }
    
 
      *side = index[SNAKE_GATE_HEIGHT-1];
  
    //avarage peek height
   
    //debug_5 = peek_value;
  
//     for(int i = 0;i<SNAKE_GATE_HEIGHT;i++){
//     printf("hist_peeks[%d]:%d\n",i,hist_peeks[i]);
//     }
 
//     for(int i = 0;i<SNAKE_GATE_HEIGHT;i++){
//       printf("hist_raw[%d]:%d hist_smooth[%d]:%d hist_peeks[%d]:%d\n",i,hist_raw[i],i,hist_smooth[i],i,hist_peeks[i]);
//     }

    return hist_peeks[index[SNAKE_GATE_HEIGHT-1]];
}

void print_side(struct image_t *im, int side){
    
    struct point_t from, to;
    from.x = side;
    from.y = 0;
    to.x = side;
    to.y = SNAKE_GATE_WIDTH;
    image_draw_line(im, &from, &to);
  
}

// Function
// Samples from the image and checks if the pixel is the right color.
// If yes, it "snakes" up and down to see if it is the side of a gate.
// If this stretch is long enough, it "snakes" also left and right.
// If the left/right stretch is also long enough, add the coords as a
// candidate square, optionally drawing it on the image.
//struct image_t *snake_gate_detection_func(struct image_t *img);
//struct image_t *snake_gate_detection_func(struct image_t *img)

//return 1 if usable detection and ls position available
int open_gate_processing(struct image_t *img,float *o_pos_x, float *o_pos_y, float *o_pos_z)
{
  int filter = 0;
  int ogate_graphics = 0;
  uint16_t i;
  int x, y, y_low, y_high, x_low1, x_high1, x_low2, x_high2, sz, szx1, szx2;
  float quality;
  struct point_t from, to;
  //struct point_t low_s1, high_s1, low_s2, high_s2;
  int n_opengates = 0;
  int y_quarter, y_m_quarter, y_third_quarter, y_five_quarter;
  int x_rand, y_rand;
  int temp_size = 0;
  int temp_bars = 0;
  int best_op_quality = 0;
  int best_bars = 0;
  int x_lmr[3];
  int x_corners[4];
  int y_corners[4];
  int y_temp_r[2];
  int found[4] = {0}; // left lower; left upper; right upper; right lower found
  int size_factor = 1.5;
  int best_loc[2];
  
  int histogram_c[SNAKE_GATE_HEIGHT] = {0};
  
  struct opengate_img temp_best_opengate;
  for (i = 0; i < n_op_samples; i++) {
	  x = rand() % img->h;
	  y = rand() % img->w;

	  if (check_color(img, x, y)) {
	    
		  //build histogram for closed gate pole avoidance
		  histogram_c[x]++;
	    
		  // snake up and down for the central bar.
		  snake_up_and_down_new(img, x, y, &y_low, &y_high);
		  sz = y_high - y_low;
		  // thickness?
		  y = (y_high + y_low)/2;

		  if (sz > min_pixel_size_o) {
			  x_lmr[0] = x;
			  x_lmr[1] = x-1;
			  x_lmr[2] = x+1;
			  y_quarter = (int) (sz/4 + y_low);
			  y_m_quarter = ((int) (-sz/4 + y_low) < 0) ? 0 : (int) (-sz/4 + y_low);
			  y_third_quarter = (int) (y_high - sz/4);
			  y_five_quarter = ((int) (y_high + sz/4) >= img->w) ? img->w-1 : (int) (y_high + sz/4);
			  for (int j = 0; j < 200; j++) {
				  x_rand = x_lmr[rand() % 3];
				  y_rand = (rand() % (y_quarter + 1 - y_m_quarter)) + y_m_quarter;
				  snake_left_and_right_new(img, x_rand, y_rand, &x_low1, &x_high1, y_temp_r);
				  if (x_high1 - x_low1 > min_pixel_size_hor) {
					  //printf("x_rand=%d, x_low1=%d, x_high1=%d, average=%d\n", x_rand, x_low1, x_high1, (x_low1 + x_high1)/2);
					  if ((x_low1 + x_high1)/2 > x_rand) {
                    	  found[3] = 1;
                    	  // y_right is atually missed out because it might be tilted. Assume to be horizontal, refinement?
                    	  x_corners[3] = x_high1;
                    	  y_corners[3] = y_temp_r[1];
                      }
                      else if ((x_low1 + x_high1)/2 < x_rand) {
                    	  found[0] = 1;
                    	  x_corners[0] = x_low1;
                    	  y_corners[0] = y_temp_r[0];
                      }
                      else {}
				  }
				  if (found[0] && found[3]) {
				      break;
				  }
			  }
			  for (int j = 0; j < 200; j++) {
				  x_rand = x_lmr[rand() %3];
				  y_rand = (rand() % (y_five_quarter + 1 - y_third_quarter)) + y_third_quarter;
				  snake_left_and_right_new(img, x_rand, y_rand, &x_low2, &x_high2, y_temp_r);
				  if (x_high2 - x_low2 > min_pixel_size_hor) {
					  //printf("x_rand=%d, x_low2=%d, x_high2=%d, average=%d\n", x_rand, x_low2, x_high2, (x_low2 + x_high2)/2);
					  if ((x_low2 + x_high2)/2 > x_rand) {
						  found[2] = 1;
						  x_corners[2] = x_high2;
						  y_corners[2] = y_temp_r[1];
					  }
					  else if ((x_low2 + x_high2)/2 < x_rand) {
						  found[1] = 1;
                          x_corners[1] = x_low2;
                          y_corners[1] = y_temp_r[0];
					  }
					  else {}
				  }
				  if (found[2] && found[1]) {
				      break;
				  }
			  }
              if ((found[0] || found[1]) || (found[3] || found[2])) {
            	  opengates[n_opengates].x = x;
            	  opengates[n_opengates].y_l = y_low;
            	  opengates[n_opengates].y_h = y_high;
            	  memcpy(&(opengates[n_opengates].found[0]), found, sizeof(int)*4);
            	  for (int j=0; j<4; j++) {
            	      if (found[j] == 1) {
            	          temp_size += abs(x - x_corners[j]);
            	          temp_bars += found[j];
            	      }
            	  }
            	  opengates[n_opengates].size_bar = (int) (temp_size/temp_bars);
            	  memcpy(&(opengates[n_opengates].x_corners[0]), x_corners, sizeof(int)*4);
            	  memcpy(&(opengates[n_opengates].y_corners[0]), y_corners, sizeof(int)*4);

            	  // quality check: only increment if better.
            	  check_opengate(img, opengates[n_opengates], &quality, &(opengates[n_opengates].n_bars));
            	  if ((quality > best_op_quality) && opengates[n_opengates].n_bars >= best_bars) {
            		  /*low_s1.x = x;
            		  high_s1.x = x;
            		  low_s1.y = y_m_quarter;
            		  high_s1.y = y_quarter;
            		  low_s2.x = x;
            		  high_s2.x = x;
            		  low_s2.y = y_third_quarter;
            		  high_s2.y = y_five_quarter;*/

                      best_op_quality = quality;
                      best_bars = opengates[n_opengates].n_bars;
                      //printf("ogate %d: n_bars = %d, quality = %f\n", n_opengates, opengates[n_opengates].n_bars, quality);
                      //printf("img-h%"PRIu16"\n", img->h);//img-hSNAKE_GATE_HEIGHT
                      //printf("img-w%"PRIu16"\n", img->w);//img-wSNAKE_GATE_WIDTH
                      n_opengates++;
            	  }
              }
              memset(found, 0, 4*sizeof(int));
              memset(x_corners, 0, 4*sizeof(int));
              memset(y_corners, 0, 4*sizeof(int));
              temp_size = 0;
              temp_bars = 0;
              if (n_opengates >= MAX_GATES) {
            	  break;
              }
		  }
	  }
  }

  if (n_opengates > 0) {
      //image_draw_line_color(img, &low_s1, &high_s1, green_color);
      //image_draw_line_color(img, &low_s2, &high_s2, green_color);
    
	  // find a better search region for the histogram since y_low and y_hight might not be accurate.
	  if (opengates[n_opengates-1].found[0] && (opengates[n_opengates-1].y_l > opengates[n_opengates-1].y_corners[0])) {
	      opengates[n_opengates-1].y_l = opengates[n_opengates-1].y_corners[0];
	  }
	  if (opengates[n_opengates-1].found[3] && (opengates[n_opengates-1].y_l > opengates[n_opengates-1].y_corners[3])) {
		  opengates[n_opengates-1].y_l = opengates[n_opengates-1].y_corners[3];
	  }
	  if (opengates[n_opengates-1].found[1] && (opengates[n_opengates-1].y_h < opengates[n_opengates-1].y_corners[1])) {
		  opengates[n_opengates-1].y_h = opengates[n_opengates-1].y_corners[1];
	  }
	  if (opengates[n_opengates-1].found[2] && (opengates[n_opengates-1].y_h < opengates[n_opengates-1].y_corners[2])) {
	  		  opengates[n_opengates-1].y_h = opengates[n_opengates-1].y_corners[2];
	  }
	  if ((~opengates[n_opengates-1].found[0]) || (~opengates[n_opengates-1].found[1])
			  || (~opengates[n_opengates-1].found[2]) || (~opengates[n_opengates-1].found[3])) {
		  ogate_histogram(img, &(opengates[n_opengates-1]), size_factor, best_loc); // before color seg!
	  }

      check_opengate(img, opengates[n_opengates-1], &(opengates[n_opengates-1].opengate_q), &(opengates[n_opengates-1].n_bars));

      //draw_opengate(img, opengates[n_opengates-1], blue_color);
      
      //printf("ogate %d: best_loc_left=%d, best_loc_right=%d\n\n", n_opengates, best_loc[0], best_loc[1]);
  }

  //color filtered version of image for overlay and debugging
  if (filter) {
      int color_count = image_yuv422_colorfilt(img, img,
    		  color_lum_min, color_lum_max,
			  color_cb_min, color_cb_max,
    		  color_cr_min, color_cr_max);
  }
  
  
  //Open gate histogram stuff 
  int center_p;
  int peek_height_o = detect_gate_side_o(histogram_c,&center_p);
  
  float undist_x, undist_y;
  float princ_x = 157.0;
  float princ_y = 32.0;
  int f_fisheye = 168;
  
  //float psi_comp = stateGetNedToBodyEulers_f()->psi;
  undistort_fisheye_point(center_p,princ_y,&undist_x,&undist_y,f_fisheye,1.150,princ_x,princ_y);
  float side_angle = atanf(undist_x/f_fisheye);
  
  // Send to position estimator
  first_stretch_psi = side_angle_o;
  first_stretch_certainty = peek_height_o;
  first_stretch_add = 1;


  //0.5m 8
  //1m 6
  //1.5m 5
  
  //printf("side_angle:%f              peek_height_o:%d\n",side_angle*57,peek_height_o);
  
  if(peek_height_o > 4)print_side(img,center_p);
  

  //printf("opengates[n_opengates-1].n_bars:%d\n",opengates[n_opengates-1].n_bars);
  // Best open gate to coordinates.
  if (n_opengates>0 && opengates[n_opengates-1].opengate_q > min_gate_quality_o &&
    opengates[n_opengates-1].found[0] && opengates[n_opengates-1].found[1] &&
    (opengates[n_opengates-1].found[2] || opengates[n_opengates-1].found[3])
  ) {
    
	  draw_opengate(img, opengates[n_opengates-1], blue_color);
    
	  struct FloatVect3 ogate_points[5];
	  struct point_t central_points[5];
	  int left;
	  central_points[0].x = opengates[n_opengates-1].x;
	  central_points[2].x = opengates[n_opengates-1].x;
	  central_points[4].x = opengates[n_opengates-1].x;

	  VECT3_ASSIGN(ogate_points[0], ogate_dist_x, 0, -1.10);
	  VECT3_ASSIGN(ogate_points[2], ogate_dist_x, 0, -1.25);
	  VECT3_ASSIGN(ogate_points[4], ogate_dist_x, 0, -2.45);
	  if (opengates[n_opengates-1].y_corners[0] < opengates[n_opengates-1].y_corners[3]) {
		  central_points[0].y = opengates[n_opengates-1].y_corners[0];
		  central_points[2].y = opengates[n_opengates-1].y_corners[3];
		  if (opengates[n_opengates-1].found[2]>0) {
			  central_points[4].y = opengates[n_opengates-1].y_corners[2];
		  } else {
			  central_points[4].y = opengates[n_opengates-1].y_corners[1];
		  }

		  central_points[1].y = central_points[0].y;
		  central_points[1].x = opengates[n_opengates-1].x - opengates[n_opengates-1].size_bar;
		  central_points[3].y = central_points[2].y;
		  central_points[3].x = opengates[n_opengates-1].x + opengates[n_opengates-1].size_bar;
		  VECT3_ASSIGN(ogate_points[1], ogate_dist_x, -0.70, -1.10);
		  VECT3_ASSIGN(ogate_points[3], ogate_dist_x, 0.70, -1.25);
		  left = 1;

	  } else {
		  central_points[0].y = opengates[n_opengates-1].y_corners[3];

		  central_points[2].y = opengates[n_opengates-1].y_corners[0];
		  if (opengates[n_opengates-1].found[1]>0) {
			  central_points[4].y = opengates[n_opengates-1].y_corners[1];
		  } else {
			  central_points[4].y = opengates[n_opengates-1].y_corners[2];
		  }

		  central_points[1].y = central_points[0].y;
		  central_points[1].x = opengates[n_opengates-1].x + opengates[n_opengates-1].size_bar;
		  central_points[3].y = central_points[2].y;
		  central_points[3].x = opengates[n_opengates-1].x - opengates[n_opengates-1].size_bar;
		  VECT3_ASSIGN(ogate_points[1], ogate_dist_x, 0.70, -1.10);
		  VECT3_ASSIGN(ogate_points[3], ogate_dist_x, -0.70, -1.25);
		  left = 0;
	  }


// 	  printf("left %d, right %d, point 0: (%d, %d); point 2: (%d, %d); point 4: (%d, %d)\n",
// 			  left, 1-left,
// 			  central_points[0].x, central_points[0].y,
// 			  central_points[2].x, central_points[2].y,
// 			  central_points[4].x, central_points[4].y);

	  if (stateGetNedToBodyEulers_f()->psi > 1.6 || stateGetNedToBodyEulers_f()->psi < -1.6) {
	      if (stateGetNedToBodyEulers_f()->psi<0) {
	         local_psi_o = stateGetNedToBodyEulers_f()->psi+3.14;
	      } else {
	  	      local_psi_o = stateGetNedToBodyEulers_f()->psi-3.14;
	      }
	  } else {
	  	  local_psi_o = stateGetNedToBodyEulers_f()->psi;
	  }

	  float x_ogate_corners[5];
	  float y_ogate_corners[5];

	  int f_fisheye = 168;
	  float k_fisheye = 1.085;

	  struct FloatRMat R, R_20, R_trans, Q_mat, I_mat, temp_mat, temp_mat_2;
	  struct FloatEulers attitude, cam_body;
	  struct FloatVect3 vec_20, p_vec, pos_vec, temp_vec, n_vec;

	  p_vec.x = 0;
	  p_vec.y = 0;
	  p_vec.z = 0;

	  MAT33_ELMT(I_mat, 0, 0) = 1;
	  MAT33_ELMT(I_mat, 0, 1) = 0;
	  MAT33_ELMT(I_mat, 0, 2) = 0;

	  MAT33_ELMT(I_mat, 1, 0) = 0;
	  MAT33_ELMT(I_mat, 1, 1) = 1;
	  MAT33_ELMT(I_mat, 1, 2) = 0;

	  MAT33_ELMT(I_mat, 2, 0) = 0;
	  MAT33_ELMT(I_mat, 2, 1) = 0;
	  MAT33_ELMT(I_mat, 2, 2) = 1;

	  MAT33_ELMT(Q_mat, 0, 0) = 0;
	  MAT33_ELMT(Q_mat, 0, 1) = 0;
	  MAT33_ELMT(Q_mat, 0, 2) = 0;

	  MAT33_ELMT(Q_mat, 1, 0) = 0;
	  MAT33_ELMT(Q_mat, 1, 1) = 0;
	  MAT33_ELMT(Q_mat, 1, 2) = 0;

	  MAT33_ELMT(Q_mat, 2, 0) = 0;
	  MAT33_ELMT(Q_mat, 2, 1) = 0;
	  MAT33_ELMT(Q_mat, 2, 2) = 0;

	  attitude.phi = stateGetNedToBodyEulers_f()->phi;
	  attitude.theta = stateGetNedToBodyEulers_f()->theta;
	  attitude.psi = local_psi_o;
	  float_rmat_of_eulers_321(&R, &attitude);

	  cam_body.phi = 0;
	  cam_body.theta = -25*(3.1415/180.0);
	  cam_body.psi = 0;
	  float_rmat_of_eulers_321(&R_20, &cam_body);

	  float undist_x, undist_y;
	  float princ_x = 157.0;
	  float princ_y = 32.0;
	  for (int i=0; i<5; i++) {
		  undistort_fisheye_point(central_points[i].x, central_points[i].y, &undist_x, &undist_y, f_fisheye, k_fisheye, princ_x, princ_y);
		  x_ogate_corners[i] = undist_x + princ_x;
		  y_ogate_corners[i] = undist_y + princ_y;
		  if (ogate_graphics) draw_cross(img, (int) x_ogate_corners[i], (int) y_ogate_corners[i], green_color);

		  //vec_from_point_ned(undist_x, undist_y, f_fisheye,&gate_vectors[i]);
		  vec_from_point_2(undist_x, undist_y, f_fisheye, &temp_vec);
		  MAT33_VECT3_MUL(vec_20, R_20, temp_vec);

		  MAT33_TRANS(R_trans, R);
		  MAT33_VECT3_MUL(n_vec, R_trans, vec_20);

		  n_vec.z = -n_vec.z;

		  double vec_norm = sqrt(VECT3_NORM2(n_vec));
		  VECT3_SDIV(n_vec, n_vec, vec_norm);
		  VECT3_VECT3_TRANS_MUL(temp_mat, n_vec, n_vec);
		  MAT33_MAT33_DIFF(temp_mat, I_mat, temp_mat);
		  MAT33_COPY(temp_mat_2, Q_mat);

		  MAT33_MAT33_SUM(Q_mat, temp_mat_2, temp_mat);
		  MAT33_VECT3_MUL(temp_vec, temp_mat, ogate_points[i]);
		  VECT3_SUM(p_vec, p_vec, temp_vec);
      }
	  MAT33_INV(temp_mat, Q_mat);
	  MAT33_VECT3_MUL(pos_vec, temp_mat, p_vec);

	  *o_pos_x = pos_vec.x;

	  float y_threshold = 4;
	  if (pos_vec.y > y_threshold) pos_vec.y = y_threshold;
	  if (pos_vec.y < -y_threshold) pos_vec.y = -y_threshold;

	  *o_pos_y = pos_vec.y;
	  *o_pos_z = pos_vec.z;
	 // printf("position x=%.2f, y=%.2f, z=%.2f\n", *o_pos_x, *o_pos_y, *o_pos_z);
  }else{
    return 0;
  }
	
	
  return 1; // detection succesfull, else return 0 before ls part
}

// my modification.
void check_opengate(struct image_t *im, struct opengate_img opengate, float *quality, int *n_bars){
	int color_count = 0;
	int n_random_samples = 30;
	float distractor_thres = 0.25;

	int x_sample;
	int y_sample;
	int x_low = (opengate.x - opengate.size_bar < 0) ? 0 : opengate.x - opengate.size_bar;
	int x_high = (opengate.x + opengate.size_bar > im->h) ? im->h-1 : opengate.x + opengate.size_bar;

	for (int i=0; i<n_random_samples; i++) {
		x_sample = (rand() % (x_high + 1 - x_low)) + x_low;
		y_sample = (rand() % (opengate.y_h + 1 - opengate.y_l)) + opengate.y_l;
		if (check_color(im, x_sample, y_sample)) {
			color_count++;
		}
	}

	if ( (float) color_count/ (float) n_random_samples > distractor_thres) {
		(*quality) = 0;
	} else {
	    int n_points = 0;
	    int n_colored_points = 0;
	    int np, nc;

	    float min_ratio_bar = 0.30;
	    (*n_bars) = 0;
	    struct point_t from, to;
        from.x = opengate.x;
        from.y = opengate.y_l;
        to.x = opengate.x;
        to.y = opengate.y_h;
        check_line(im, from, to, &np, &nc);
        n_points += np;
        n_colored_points += nc;

        if (opengate.x_corners[0] > 0) {
            from.x = opengate.x_corners[0];
            from.y = opengate.y_corners[0];
            to.x = opengate.x;
            to.y = opengate.y_corners[0];
            check_line(im, from, to, &np, &nc);
            if ((float) nc/ (float) np >= min_ratio_bar) {
     	        (*n_bars)++;
            }
            n_points += np;
            n_colored_points += nc;
        }

        if (opengate.x_corners[1] > 0) {
            from.x = opengate.x_corners[1];
            from.y = opengate.y_corners[1];
            to.x = opengate.x;
            to.y = opengate.y_corners[1];
            check_line(im, from, to, &np, &nc);
            if ((float) nc/ (float) np >= min_ratio_bar) {
      	        (*n_bars)++;
            }
            n_points += np;
            n_colored_points += nc;
        }

        if (opengate.x_corners[2] > 0) {
            from.x = opengate.x_corners[2];
            from.y = opengate.y_corners[2];
            to.x = opengate.x;
            to.y = opengate.y_corners[2];
            check_line(im, from, to, &np, &nc);
            if ((float) nc/ (float) np >= min_ratio_bar) {
      	        (*n_bars)++;
            }
            n_points += np;
            n_colored_points += nc;
        }

        if (opengate.x_corners[3] > 0) {
            from.x = opengate.x_corners[3];
            from.y = opengate.y_corners[3];
            to.x = opengate.x;
            to.y = opengate.y_corners[3];
            check_line(im, from, to, &np, &nc);
            if ((float) nc/ (float) np >= min_ratio_bar) {
      	        (*n_bars)++;
            }
            n_points += np;
            n_colored_points += nc;
        }

        if (n_points == 0){
    	    (*quality) = 0;
        } else {
    	    (*quality) = (((float) n_colored_points)/ ((float) n_points));
        }
	}
}

void draw_opengate(struct image_t *im, struct opengate_img opengate, uint8_t* color) {
	struct point_t from, to;
	from.x = opengate.x;
	from.y = opengate.y_l;
	to.x = opengate.x;
	to.y = opengate.y_h;
	image_draw_line_color(im, &from, &to, color);
	//if (opengate.x_corners[0] > 0) {
	if (opengate.found[0]==1) {
	from.x = opengate.x_corners[0];
	from.y = opengate.y_corners[0];
	to.x = opengate.x;
	to.y = opengate.y_corners[0];
	image_draw_line_color(im, &from, &to, color);
	}
	if (opengate.found[0]==2) {
	from.x = opengate.x_corners[0];
	from.y = opengate.y_corners[0];
	to.x = opengate.x;
	to.y = opengate.y_corners[0];
	image_draw_line(im, &from, &to);
	}

	//if (opengate.x_corners[1] > 0) {
	if (opengate.found[1]==1) {
	from.x = opengate.x_corners[1];
	from.y = opengate.y_corners[1];
	to.x = opengate.x;
	to.y = opengate.y_corners[1];
	image_draw_line_color(im, &from, &to, color);
	}
	if (opengate.found[1]==2) {
	from.x = opengate.x_corners[1];
	from.y = opengate.y_corners[1];
	to.x = opengate.x;
	to.y = opengate.y_corners[1];
	image_draw_line(im, &from, &to);
	}

	//if (opengate.x_corners[2] > 0) {
	if (opengate.found[2]==1) {
	from.x = opengate.x_corners[2];
    from.y = opengate.y_corners[2];
	to.x = opengate.x;
	to.y = opengate.y_corners[2];
	image_draw_line_color(im, &from, &to, color);
	}
	if (opengate.found[2]==2) {
	from.x = opengate.x_corners[2];
	from.y = opengate.y_corners[2];
	to.x = opengate.x;
	to.y = opengate.y_corners[2];
	image_draw_line(im, &from, &to);
	}

	//if (opengate.x_corners[3] > 0) {
	if (opengate.found[3]==1) {
	from.x = opengate.x_corners[3];
	from.y = opengate.y_corners[3];
	to.x = opengate.x;
	to.y = opengate.y_corners[3];
	image_draw_line_color(im, &from, &to, color);
	}
	if (opengate.found[3]==2) {
	from.x = opengate.x_corners[3];
	from.y = opengate.y_corners[3];
	to.x = opengate.x;
	to.y = opengate.y_corners[3];
	image_draw_line(im, &from, &to);
	}
}

void snake_up_and_down_new(struct image_t *im, int x, int y, int *y_low, int *y_high)
{
  int done = 0;
  int x_initial = x;
  int no_pixel_gap = 0;
  (*y_low) = y;

  // snake towards negative y (down?)
  while ((*y_low) > 0 && !done) {
    if (check_color(im, x, (*y_low) - 1)) {
      (*y_low)--;
      no_pixel_gap = 1;
    } else if (x+1 < im->h && check_color(im, x + 1, (*y_low) - 1)) {
      x++;
      (*y_low)--;
      no_pixel_gap = 1;
    } else if (x-1 >= 0 && check_color(im, x - 1, (*y_low) - 1)) {
      x--;
      (*y_low)--;
      no_pixel_gap = 1;
    } else if (~no_pixel_gap && check_color(im, x, (*y_low)-2)) {
    	(*y_low) -= 2;
    	no_pixel_gap = 1;
    } else if (~no_pixel_gap && x+1 < im->h && check_color(im, x + 1, (*y_low)-2)) {
    	x++;
    	(*y_low) -=2;
    	no_pixel_gap = 1;
    } else if (~no_pixel_gap && x-1 > 0 && check_color(im, x - 1, (*y_low)-2)) {
    	x--;
    	(*y_low) -=2;
    	no_pixel_gap = 1;
    } else {
      done = 1;
    }
    no_pixel_gap = 0;
  }

  x = x_initial;
  (*y_high) = y;
  done = 0;
  // snake towards positive y (up?)
  // while ((*y_high) < im->h - 1 && !done) {
  while ((*y_high) < im->w - 1 && !done) {

    if (check_color(im, x, (*y_high) + 1)) {
      (*y_high)++;
      no_pixel_gap = 1;
    //    } else if (x < im->w - 1 && check_color(im, x + 1, (*y_high) + 1)) {
    } else if (x < im->h - 1 && check_color(im, x + 1, (*y_high) + 1)) {
      x++;
      (*y_high)++;
      no_pixel_gap = 1;
    } else if (x > 0 && check_color(im, x - 1, (*y_high) + 1)) {
      x--;
      (*y_high)++;
      no_pixel_gap = 1;
    } else if (~no_pixel_gap && check_color(im, x, (*y_high) + 2)) {
    	(*y_high) += 2;
    	no_pixel_gap = 1;
    } else if (~no_pixel_gap && check_color(im, x + 1, (*y_high) + 2)) {
    	x++;
    	(*y_high) += 2;
    	no_pixel_gap = 1;
    } else if (~no_pixel_gap && check_color(im, x - 1, (*y_high) + 2)) {
    	x--;
    	(*y_high) += 2;
    } else {
      done = 1;
    }
    no_pixel_gap = 0;
  }
}

void snake_left_and_right_new(struct image_t *im, int x, int y, int *x_low, int *x_high, int y_result[])
{
  int done = 0;
  int no_pixel_gap = 0;
  int y_initial = y;
  (*x_low) = x;

  // snake towards negative x (left)
  while ((*x_low) > 0 && !done) {
    if (check_color(im, (*x_low) - 1, y)) {
      (*x_low)--;
      no_pixel_gap = 1;
    // } else if (y < im->h - 1 && check_color(im, (*x_low) - 1, y + 1)) {
    } else if (y < im->w - 1 && check_color(im, (*x_low) - 1, y + 1)) {
      y++;
      (*x_low)--;
      no_pixel_gap = 1;
    } else if (y > 0 && check_color(im, (*x_low) - 1, y - 1)) {
      y--;
      (*x_low)--;
      no_pixel_gap = 1;
    } else if (~no_pixel_gap && check_color(im, (*x_low) - 2, y)) {
    	(*x_low)-=2;
    	no_pixel_gap = 1;
    } else if (y < im->w -1 && ~no_pixel_gap && check_color(im, (*x_low) - 2, y + 1)) {
    	y++;
    	(*x_low)-=2;
    	no_pixel_gap = 1;
    } else if (y > 0 && ~no_pixel_gap && check_color(im, (*x_low) - 2, y - 1)) {
    	y--;
    	(*x_low)-=2;
    	no_pixel_gap = 1;
    } else {
      done = 1;
    }
    no_pixel_gap = 0;
  }
  y_result[0] = y; // y to the left side
  y = y_initial;
  (*x_high) = x;
  done = 0;
  // snake towards positive x (right)
  // while ((*x_high) < im->w - 1 && !done) {
  // y+1? Problematic.
  while ((*x_high) < im->h - 1 && !done) {

    if (check_color(im, (*x_high) + 1, y)) {
      (*x_high)++;
      no_pixel_gap = 1;
    // } else if (y < im->h - 1 && check_color(im, (*x_high) + 1, y++)) {
    } else if (y < im->w - 1 && check_color(im, (*x_high) + 1, y + 1)) {
      y++;
      (*x_high)++;
      no_pixel_gap = 1;
    } else if (y > 0 && check_color(im, (*x_high) + 1, y - 1)) {
      y--;
      (*x_high)++;
      no_pixel_gap = 1;
    } else if (~no_pixel_gap && check_color(im, (*x_high) + 2, y)) {
    	(*x_high)+=2;
    	no_pixel_gap = 1;
    } else if (y < im->w - 1 && ~no_pixel_gap && check_color(im, (*x_high) + 2, y + 1)) {
    	y++;
    	(*x_high)+=2;
    	no_pixel_gap = 1;
    } else if (y > 0 && ~no_pixel_gap && check_color(im, (*x_high) + 2, y - 1)) {
    	y--;
    	(*x_high)+=2;
    	no_pixel_gap = 1;
    } else {
      done = 1;
    }
    no_pixel_gap = 0;
  }
  y_result[1] = y; // y to the right side
  y = y_initial;
}

