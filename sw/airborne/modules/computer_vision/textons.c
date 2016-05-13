/*
 * Copyright (C) 2016, Hann Woei Ho, Guido de Croon
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
 * @file modules/computer_vision/textons.c
 *
 * Takes an image and represents the texture and colors in the image with a texton histogram.
 * A texton is a cluster centroid in a space populated by image patches. First, this code
 * learns or loads a texton dictionary. Then, for each incoming image, patches are sampled from
 * the image, compared to textons in the dictionary, and the closest texton is identified,
 * augmenting the corresponding bin in the histogram.
 */

// Own header
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/textons.h"

#include <stdio.h>

float ****dictionary;
uint32_t learned_samples;
uint8_t dictionary_initialized;

// Function
struct image_t* texton_func(struct image_t* img);
struct image_t* texton_func(struct image_t* img)
{
  


  return img; // Colorfilter did not make a new image
}

void colorfilter_init(void)
{
  cv_add(texton_func);
}

