
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

// snake-gate:
int snake_gate_detection(struct image_t *img); // main function to be called externally.
int check_color(struct image_t *im, int x, int y);
void snake_up_and_down(struct image_t *im, int x, int y, int *y_low, int *y_high);
void snake_left_and_right(struct image_t *im, int x, int y, int *x_low, int *x_high);
void draw_gate(struct image_t *im, struct gate_img gate);
void draw_gate_color(struct image_t *im, struct gate_img gate, uint8_t *color);
void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int *n_points, int *n_colored_points);
void check_gate_initial(struct image_t *im, struct gate_img gate, float *quality, int *sides);
void check_gate_outline(struct image_t *im, struct gate_img gate, float *quality, int *n_sides);
