/*
 * Copyright (C) 2016 Hann Woei Ho, Guido de Croon
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
 * @file modules/computer_vision/textons.h
 *
 * Takes an image and represents the texture and colors in the image with a texton histogram.
 * A texton is a cluster centroid in a space populated by image patches. First, this code
 * learns or loads a texton dictionary. Then, for each incoming image, patches are sampled from
 * the image, compared to textons in the dictionary, and the closest texton is identified,
 * augmenting the corresponding bin in the texton histogram.
 *
 */

#ifndef TEXTONS_H
#define TEXTONS_H

#include <stdint.h>


extern float *texton_distribution; // main outcome of the image processing: the distribution of textons in the image
extern uint8_t load_dictionary;
extern uint8_t load_model;
extern float alpha;
extern uint8_t n_textons;
extern uint8_t patch_size; // TODO: Why was there the comment "use even number for YUV image" in the original code? Should it be even?
extern uint32_t n_learning_samples;
extern uint32_t n_samples_image;
extern uint8_t FULL_SAMPLING;
extern uint32_t border_width;
extern uint32_t border_height;
extern uint8_t dictionary_ready;
extern uint8_t dictionary_number;

// Module functions
extern void textons_init(void);


#endif /* TEXTONS_H */
